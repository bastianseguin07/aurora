"""Surface-aware comparison: planar regions and local normal projection.

Geometry and overlap gates precede change thresholds. This is inspired by
M3C2's local cylinders; quality/variation indicators are not calibrated confidence
intervals. Original point coordinates are never transformed here.
"""
from dataclasses import dataclass, asdict
import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree, ConvexHull, QhullError
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components


@dataclass
class SurfaceParams:
    geometry_voxel_m: float = .015
    core_step_m: float = .04
    normal_radius_m: float = .07
    projection_radius_m: float = .06
    search_depth_m: float = .60
    normal_angle_deg: float = 20.
    min_neighbors: int = 8
    registration_allowance_m: float = .01

    def validate(self):
        values = asdict(self)
        if not all(np.isfinite(v) for v in values.values()):
            raise ValueError('Los parámetros de superficies deben ser finitos.')
        if not (0 < self.geometry_voxel_m <= self.projection_radius_m / 2
                and self.core_step_m >= self.geometry_voxel_m
                and self.normal_radius_m >= 2 * self.geometry_voxel_m
                and self.search_depth_m > 0 and self.registration_allowance_m >= 0
                and 0 < self.normal_angle_deg < 45
                and 6 <= self.min_neighbors <= 60 and int(self.min_neighbors) == self.min_neighbors):
            raise ValueError('Parámetros de superficies incompatibles: revisa escalas, búsqueda y soporte.')


@dataclass
class SurfaceComparison:
    signed: np.ndarray
    valid: np.ndarray
    method: np.ndarray       # 0=no measurement, 1=planar face, 2=local normal
    quality: np.ndarray      # 0=no measurement, 1=medium, 2=high; not probabilities
    variation: np.ndarray
    region: np.ndarray
    reason: np.ndarray      # 0=valid, 1=no support, 2=ambiguous, 3=no overlap, 4=edge/noise
    regions: list
    params: dict
    unmatched_before: np.ndarray


def _representatives(points, step):
    """One actual sample per voxel; never average intersecting surfaces."""
    keys = np.floor(points / step).astype(np.int64)
    _, indices = np.unique(keys, axis=0, return_index=True)
    return np.sort(indices)


def _dominant_plane(points, tolerance=.012):
    """Deterministic RANSAC; never reseed Open3D's process-global generator."""
    sample = points[np.linspace(0, len(points)-1, min(len(points), 4096), dtype=int)]
    rng = np.random.default_rng(1729)
    triples = sample[rng.integers(0, len(sample), size=(600, 3))]
    normals = np.cross(triples[:, 1]-triples[:, 0], triples[:, 2]-triples[:, 0])
    norms = np.linalg.norm(normals, axis=1)
    good = norms > 1e-10
    if not good.any():
        return None
    normals = normals[good] / norms[good, None]
    offsets = -np.einsum('ij,ij->i', triples[good, 0], normals)
    best_count, best_error, plane = 0, np.inf, None
    for start in range(0, len(normals), 64):
        n, d = normals[start:start+64], offsets[start:start+64]
        residual = np.abs(np.einsum('ij,kj->ik', sample, n, optimize=False) + d)
        inside = residual <= tolerance
        counts = inside.sum(axis=0)
        errors = np.sum(np.where(inside, residual**2, 0), axis=0) / np.maximum(counts, 1)
        candidate = np.lexsort((errors, -counts))[0]
        if counts[candidate] > best_count or (counts[candidate] == best_count and errors[candidate] < best_error):
            best_count, best_error = counts[candidate], errors[candidate]
            plane = np.r_[n[candidate], d[candidate]]
    return plane


def _model(points, cfg):
    indices = _representatives(points, cfg.geometry_voxel_m)
    xyz = points[indices]
    tree = cKDTree(xyz)
    cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz))
    normals = np.zeros_like(xyz)
    sigma = np.full(len(xyz), np.inf)
    best = np.full(len(xyz), np.inf)
    # Small neighborhoods preserve small faces and avoid floor/wall mixtures;
    # larger ones support sparse samples. Select by planarity and support.
    for scale in (.65, 1., 1.5):
        radius = cfg.normal_radius_m * scale
        cloud.estimate_covariances(o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=60))
        values, axes = np.linalg.eigh(np.asarray(cloud.covariances))
        values = np.maximum(values, 0)
        neighbors = tree.query(xyz, k=max(24, cfg.min_neighbors), distance_upper_bound=radius)[0]
        number = np.sum(np.isfinite(neighbors), axis=1)
        valid_scale = ((number >= cfg.min_neighbors) & (values[:, 1] > .004 ** 2)
                       & (values[:, 0] < .3 * values[:, 1]) & (values[:, 0] < .025 ** 2))
        score = values[:, 0] / np.maximum(values[:, 1], 1e-12) + .02 * cfg.min_neighbors / np.maximum(number, 1)
        choose = valid_scale & (score < best)
        normals[choose] = axes[choose, :, 0]
        sigma[choose] = np.sqrt(values[choose, 0])
        best[choose] = score[choose]
    valid = np.isfinite(best)
    # A small flat target can have noisy local normals simply because each
    # neighborhood contains few scan rows. Stabilize a genuinely dominant
    # plane, but only at samples whose local orientation agrees with it.
    if len(xyz) >= 30:
        plane = _dominant_plane(xyz)
        n_plane = np.asarray(plane[:3]) if plane is not None else np.zeros(3)
        inlier = np.abs(xyz @ n_plane + plane[3]) <= .012 if plane is not None else np.zeros(len(xyz), bool)
        if inlier.mean() >= .6:
            points_on_plane = xyz[inlier]
            center = points_on_plane.mean(axis=0)
            _, _, vectors = np.linalg.svd(points_on_plane - center, full_matrices=False)
            n_plane = vectors[-1]
            agree = inlier & valid & (np.abs(normals @ n_plane) >= np.cos(np.deg2rad(35)))
            normals[agree] = n_plane
            sigma[agree] = np.sqrt(np.mean(((points_on_plane - center) @ n_plane) ** 2))
    normals[np.einsum('ij,ij->i', normals, -xyz) < 0] *= -1
    # Exclude neighborhoods crossing a sharp corner before growing regions.
    nd, ni = tree.query(xyz, k=min(12, len(xyz)), distance_upper_bound=cfg.normal_radius_m * .7)
    if ni.ndim == 1:
        ni, nd = ni[:, None], nd[:, None]
    usable = ni < len(xyz)
    safe = np.minimum(ni, len(xyz) - 1)
    usable &= valid[safe]
    angles_ok = np.abs(np.einsum('ij,ikj->ik', normals, normals[safe])) >= np.cos(np.deg2rad(35))
    edge = np.sum(usable & ~angles_ok, axis=1) > np.maximum(2, .25 * usable.sum(axis=1))
    valid &= ~edge
    pairs = tree.query_pairs(cfg.geometry_voxel_m * 2.8, output_type='ndarray')
    if len(pairs):
        a, b = pairs.T
        delta = xyz[b] - xyz[a]
        compatible = (valid[a] & valid[b]
                      & (np.abs(np.einsum('ij,ij->i', normals[a], normals[b])) >= np.cos(np.deg2rad(cfg.normal_angle_deg)))
                      & (np.maximum(np.abs(np.einsum('ij,ij->i', delta, normals[a])),
                                    np.abs(np.einsum('ij,ij->i', delta, normals[b]))) < np.maximum(.015, 3 * np.maximum(sigma[a], sigma[b]))))
        pairs = pairs[compatible]
    graph = coo_matrix((np.ones(len(pairs)), (pairs[:, 0], pairs[:, 1])), shape=(len(xyz), len(xyz)))
    _, labels = connected_components(graph, directed=False)
    counts = np.bincount(labels)
    valid &= counts[labels] >= 12
    labels[~valid] = -1
    summaries = {}
    for label in np.unique(labels[valid]):
        mask = labels == label
        face = xyz[mask]
        center = face.mean(axis=0)
        _, _, vectors = np.linalg.svd(face - center, full_matrices=False)
        n = vectors[-1]
        if n @ -center < 0:
            n = -n
        spread = np.sqrt(np.mean(((face - center) @ n) ** 2))
        angle = np.degrees(np.arccos(np.clip(np.abs(normals[mask] @ n), 0, 1)))
        summaries[int(label)] = dict(center=center, normal=n,
                                    planar=bool(spread <= .012 and np.percentile(angle, 90) <= cfg.normal_angle_deg))
        if summaries[int(label)]['planar']:
            basis = _basis(n)
            try:
                hull = ConvexHull((face - center) @ basis)
                summaries[int(label)].update(footprint_basis=basis, footprint_equations=hull.equations)
            except QhullError:
                summaries[int(label)]['planar'] = False
    return dict(xyz=xyz, tree=tree, normals=normals, sigma=sigma,
                valid=valid, labels=labels, summaries=summaries)


def _basis(normal):
    axis = np.eye(3)[np.argmin(np.abs(normal))]
    u = np.cross(normal, axis)
    u /= np.linalg.norm(u)
    return np.column_stack((u, np.cross(normal, u)))


def _supported(points, center, normal):
    uv = (points - center) @ _basis(normal)
    try:
        hull = ConvexHull(uv)
        return bool(np.all(hull.equations[:, 2] <= .001))
    except QhullError:
        return False


def _footprint_mask(reference, queries, normal):
    basis = _basis(normal)
    center = reference.mean(axis=0)
    try:
        hull = ConvexHull((reference - center) @ basis)
    except QhullError:
        return np.zeros(len(queries), dtype=bool)
    uv = (queries - center) @ basis
    keep = np.ones(len(queries), dtype=bool)
    for equation in hull.equations:
        keep &= uv @ equation[:2] + equation[2] <= .001
    return keep


def _robust_sigma(values):
    return float(1.4826 * np.median(np.abs(values - np.median(values))))


def compare_surfaces(before, after, cfg=None, log=lambda _: None):
    cfg = cfg or SurfaceParams()
    cfg.validate()
    before, after = np.asarray(before, dtype=float), np.asarray(after, dtype=float)
    for points in (before, after):
        if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all() or len(points) < 30:
            raise ValueError('Se necesitan al menos 30 puntos XYZ finitos por captura.')
    log('Detectando superficies y normales locales ANTES...')
    base = _model(before, cfg)
    log('Detectando superficies y normales locales DESPUÉS...')
    updated = _model(after, cfg)
    size = len(after)
    out = SurfaceComparison(np.full(size, np.nan), np.zeros(size, bool), np.zeros(size, np.uint8),
                            np.zeros(size, np.uint8), np.full(size, np.nan), np.full(size, -1, np.int32),
                            np.full(size, 4, np.uint8), [], asdict(cfg), before.copy())
    if not np.any(updated['valid']) or not np.any(base['valid']):
        return out
    valid_geometry = np.flatnonzero(updated['valid'])
    cores = valid_geometry[_representatives(updated['xyz'][valid_geometry], cfg.core_step_m)]
    # Assign each captured sample to a nearby representative of its own surface.
    distance, geometry_idx = updated['tree'].query(after)
    source_label = updated['labels'][geometry_idx]
    assignment = np.full(size, -1, int)
    for label in np.unique(source_label[source_label >= 0]):
        ids = cores[updated['labels'][cores] == label]
        if not len(ids):
            continue
        raw = np.flatnonzero((source_label == label) & (distance <= cfg.geometry_voxel_m * 1.8))
        d, nearest = cKDTree(updated['xyz'][ids]).query(after[raw])
        good = d <= cfg.core_step_m * 1.8
        assignment[raw[good]] = ids[nearest[good]]
    order = np.argsort(assignment)
    unique, starts = np.unique(assignment[order], return_index=True)
    groups = {int(key): order[start:end] for key, start, end in zip(unique, starts, np.r_[starts[1:], len(order)]) if key >= 0}
    cosine = np.cos(np.deg2rad(cfg.normal_angle_deg))
    step = cfg.projection_radius_m * 1.5
    line_t = np.linspace(-cfg.search_depth_m, cfg.search_depth_m,
                         max(3, int(np.ceil(2 * cfg.search_depth_m / step)) + 1))
    ball_radius = np.hypot(cfg.projection_radius_m, (line_t[1] - line_t[0]) / 2)
    covered_before = np.zeros(len(base['xyz']), dtype=bool)
    log(f'Buscando correspondencias en {len(groups):,} regiones locales...')
    for count, (core, raw_indices) in enumerate(groups.items()):
        if count and count % 1000 == 0:
            log(f'Comparando regiones: {count:,}/{len(groups):,}...')
        p, n = updated['xyz'][core], updated['normals'][core]
        label_after = int(updated['labels'][core])
        row = dict(id=int(core), after_surface=label_after, center=p.tolist(),
                   method='none', quality='none', reason='no_support', n_after=0, n_before=0)
        out.region[raw_indices] = core
        out.reason[raw_indices] = 1
        neighbors = base['tree'].query_ball_point(p + line_t[:, None] * n, ball_radius)
        candidate = np.unique(np.concatenate([np.asarray(x, int) for x in neighbors]))
        if len(candidate):
            delta = base['xyz'][candidate] - p
            axial = delta @ n
            tangent2 = np.einsum('ij,ij->i', delta, delta) - axial ** 2
            keep = (base['valid'][candidate] & (np.abs(axial) <= cfg.search_depth_m)
                    & (tangent2 <= cfg.projection_radius_m ** 2)
                    & (np.abs(base['normals'][candidate] @ n) >= cosine))
            candidate = candidate[keep]
        labels, numbers = np.unique(base['labels'][candidate], return_counts=True)
        choices = labels[numbers >= cfg.min_neighbors]
        if len(choices) != 1:
            if len(choices) > 1:
                row['reason'] = 'ambiguous'
                out.reason[raw_indices] = 2
            out.regions.append(row)
            continue
        label_before = int(choices[0])
        candidate = candidate[base['labels'][candidate] == label_before]
        reference = base['xyz'][candidate]
        summary = base['summaries'][label_before]
        normal = summary['normal'].copy() if summary['planar'] else base['normals'][candidate].mean(axis=0)
        magnitude = np.linalg.norm(normal)
        if not np.isfinite(magnitude) or magnitude < .5:
            row['reason'] = 'ambiguous'; out.reason[raw_indices] = 2; out.regions.append(row); continue
        normal /= magnitude
        if normal @ n < 0:
            normal = -normal
        projected = (reference - p) @ normal
        sorted_t = np.sort(projected)
        splits = np.flatnonzero(np.diff(sorted_t) > max(.04, 4 * float(np.max(base['sigma'][candidate])))) + 1
        clusters = np.split(sorted_t, splits)
        if sum(len(x) >= cfg.min_neighbors for x in clusters) > 1:
            row['reason'] = 'ambiguous'; out.reason[raw_indices] = 2; out.regions.append(row); continue
        local_ids = updated['tree'].query_ball_point(p, cfg.projection_radius_m)
        local_ids = np.asarray(local_ids, int)
        local_ids = local_ids[updated['labels'][local_ids] == label_after]
        if len(local_ids) < cfg.min_neighbors:
            out.regions.append(row); continue
        observed = updated['xyz'][local_ids]
        planar = summary['planar'] and updated['summaries'][label_after]['planar']
        if not planar:
            observed = observed[_footprint_mask(reference, observed, normal)]
            reference_mask = _footprint_mask(observed, reference, normal) if len(observed) >= 3 else np.zeros(len(reference), bool)
            reference, candidate = reference[reference_mask], candidate[reference_mask]
        if min(len(reference), len(observed)) < cfg.min_neighbors:
            row['reason'] = 'no_overlap'; out.reason[raw_indices] = 3; out.regions.append(row); continue
        if not _supported(reference, p, normal):
            row['reason'] = 'no_overlap'; out.reason[raw_indices] = 3; out.regions.append(row); continue
        angle = float(np.degrees(np.arccos(np.clip(abs(normal @ n), 0, 1))))
        if angle > cfg.normal_angle_deg:
            row['reason'] = 'normal_mismatch'; out.reason[raw_indices] = 4; out.regions.append(row); continue
        before_t, after_t = (reference - p) @ normal, (observed - p) @ normal
        spread_before, spread_after = _robust_sigma(before_t), _robust_sigma(after_t)
        variation = (cfg.registration_allowance_m + max(spread_before, spread_after)
                     + 2 * np.sqrt(spread_before ** 2 / len(reference) + spread_after ** 2 / len(observed)))
        code = 1 if planar else 2
        quality = 2 if min(len(reference), len(observed)) >= 16 and angle < 10 and max(spread_before, spread_after) < .012 else 1
        delta = after[raw_indices] - p
        if planar:
            uv = (after[raw_indices] - summary['center']) @ summary['footprint_basis']
            supported = np.ones(len(raw_indices), bool)
            for equation in summary['footprint_equations']:
                supported &= uv @ equation[:2] + equation[2] <= .001
        else:
            supported = _footprint_mask(reference, after[raw_indices], normal)
        axial = delta @ n
        good = ((np.abs(axial) <= max(.025, 3 * float(updated['sigma'][core]))) & supported
                & ((np.einsum('ij,ij->i', delta, delta) - axial ** 2) <= cfg.projection_radius_m ** 2))
        accepted = raw_indices[good]
        if not len(accepted):
            row['reason'] = 'no_overlap'; out.reason[raw_indices] = 3; out.regions.append(row); continue
        if planar:
            signed = (after[accepted] - summary['center']) @ normal
        else:
            signed = np.full(len(accepted), np.median(after_t) - np.median(before_t))
        out.signed[accepted] = signed
        out.valid[accepted] = True
        out.method[accepted] = code
        out.quality[accepted] = quality
        out.variation[accepted] = variation
        out.reason[accepted] = 0
        if len(accepted):
            covered_before[candidate] = True
        row.update(before_surface=label_before, method='plane' if planar else 'local_normal',
                   quality='high' if quality == 2 else 'medium', reason='valid',
                   n_after=len(observed), n_before=len(reference), normal=normal.tolist(),
                   normal_angle_deg=angle, signed_m=float(np.median(signed)) if len(signed) else None,
                   variation_m=float(variation), raw_points=len(accepted))
        out.regions.append(row)
    before_distance, before_geometry = base['tree'].query(before)
    covered = covered_before[before_geometry] & (before_distance <= cfg.geometry_voxel_m * 1.8)
    out.unmatched_before = before[~covered].copy()
    return out
