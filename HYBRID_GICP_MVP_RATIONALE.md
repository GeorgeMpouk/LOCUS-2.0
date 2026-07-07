# Hybrid GICP MVP: Rationale for the Code Changes

## 1. Purpose

The original LOCUS 2.0 implementation selects one covariance strategy for an
entire source or target point cloud:

- Use the normals already stored in `PointXYZINormal`.
- Recompute every covariance using the standard GICP kNN/PCA procedure.

The MVP adds a third, per-point strategy:

```text
if the stored normal is reliable:
    construct the covariance from that normal
else:
    compute the covariance using standard GICP
```

The implementation intentionally changes only covariance selection. It does not
change correspondence search, the GICP objective, the optimizer, or the
transformation update.

The current code is compared against `upstream/main` at commit `84c0fed`.

## 2. Definition of a Reliable Normal

The available point type is `pcl::PointXYZINormal`. It stores:

- `normal_x`, `normal_y`, and `normal_z`
- `curvature`

For this MVP, a stored normal is considered reliable when:

```text
all normal components are finite
and curvature is finite
and the normal has nonzero length
and 0 <= curvature <= hybrid_max_curvature
```

The default threshold is:

```yaml
hybrid_max_curvature: 0.03
```

PCL curvature is the smallest local covariance eigenvalue divided by the sum of
the eigenvalues. Low curvature generally indicates that the point's local
neighbourhood supports a stable surface normal.

This is deliberately a simple gate. It avoids running another PCA merely to
decide whether PCA should be run as the fallback.

## 3. Resulting Covariance Paths

### Reliable stored normal

The code uses LOCUS's existing normal-derived covariance implementation:

```text
CovarianceFromStoredNormal(point)
```

Before constructing the covariance, the normal is normalized. This prevents a
finite but non-unit normal from scaling the resulting covariance.

### Unreliable stored normal

The code retains the repository's existing standard GICP procedure:

1. Find the point's `k_correspondences_` nearest neighbours.
2. Compute the empirical neighbourhood covariance.
3. Compute its SVD.
4. Retain the SVD basis.
5. Replace the two largest singular values with `1`.
6. Replace the smallest singular value with `gicp_epsilon_`.
7. Reconstruct the covariance.

In matrix form, the fallback remains:

```text
C = U * diag(1, 1, epsilon) * U^T
```

No shape-aware or eigenvalue-preserving covariance model was added.

## 4. Changes by File

### `point_cloud_filter/src/normal_computation.cc`

#### Change

The curvature produced by PCL normal estimation is copied into the output
`pcl::PointXYZINormal`:

```cpp
point.curvature = normals->at(i).curvature;
```

#### Reason

The original code copied the normal components but discarded their associated
curvature. The per-point gate therefore had no inexpensive quality value to
inspect later.

Preserving curvature allows the registration code to assess the stored normal
without repeating the normal-estimation neighbourhood search.

### `multithreaded_gicp/include/multithreaded_gicp/gicp.h`

#### Added `<cmath>` and `<string>`

`<cmath>` is required for `std::isfinite` and `std::sqrt`. `<string>` is
required by the new string-based covariance mode setters.

#### Added `CovarianceMode`

```cpp
enum class CovarianceMode {
  NORMALS,
  RECOMPUTE,
  HYBRID_NORMAL_QUALITY
};
```

Reason:

- `NORMALS` preserves the LOCUS normal-only path.
- `RECOMPUTE` preserves the original full GICP recomputation path.
- `HYBRID_NORMAL_QUALITY` enables the new per-point decision gate.

An enum makes the three choices explicit. A boolean can represent only the two
original modes.

#### Added source and target mode members

```cpp
CovarianceMode source_covariance_mode_;
CovarianceMode target_covariance_mode_;
```

Reason:

Source and target clouds can require different policies. The MVP enables the
hybrid mode for the accumulated localization target map while preserving the
legacy source-scan behavior.

#### Added `hybrid_max_curvature_`

```cpp
double hybrid_max_curvature_;
```

It defaults to `0.03`.

Reason:

The threshold must be configurable so that it can be tuned and evaluated
against APE and runtime instead of being permanently hard-coded.

#### Updated the legacy boolean setters

```cpp
RecomputeTargetCovariance(bool)
RecomputeSourceCovariance(bool)
```

These functions now also select either `RECOMPUTE` or `NORMALS`.

Reason:

Existing configuration files and callers still use the original booleans.
Mapping the booleans onto the enum preserves their old behavior.

#### Added string mode setters

```cpp
SetSourceCovarianceMode(const std::string&)
SetTargetCovarianceMode(const std::string&)
```

Reason:

ROS YAML parameters are strings. These setters provide the bridge from the
configuration value to the internal enum.

The value `legacy` intentionally performs no override. This means the preceding
legacy boolean setter remains authoritative for old configurations.

#### Added mode parsing

Accepted values are:

```text
normals
recompute
hybrid
hybrid_planarity
```

`hybrid_planarity` remains as a compatibility alias for configuration created
during the earlier version of the MVP. Unknown values produce a warning and
fall back to `NORMALS`.

#### Added threshold validation

```cpp
SetHybridNormalCurvatureThreshold(double)
```

The setter rejects negative and non-finite thresholds.

Reason:

A bad parameter should not silently cause every normal to pass or fail. Keeping
the previous valid value gives deterministic fallback behavior.

#### Added `HasReliableStoredNormal`

The `PointF` overload checks finite values, nonzero normal magnitude, and the
curvature threshold.

The generic template returns `false`.

Reason:

Only point types that actually provide the LOCUS `PointF` normal and curvature
layout can use the stored-normal branch safely. Other point types automatically
use standard GICP.

#### Added `CovarianceFromStoredNormal`

The `PointF` overload normalizes the stored normal and calls the existing LOCUS
helper `PCLTwoPlaneVectorsFromNormal`.

The generic template returns identity but is never selected by the reliability
gate.

Reason:

This reuses the repository's established normal covariance construction rather
than introducing a second implementation of the same model.

#### Changed `computeCovariances` parameter

The final argument changed from:

```cpp
bool recompute
```

to:

```cpp
CovarianceMode covariance_mode
```

Reason:

The function must distinguish three behaviors, which cannot be represented by
one boolean.

### `multithreaded_gicp/include/multithreaded_gicp/gicp.hpp`

#### Updated the normal-only branch

The original condition used `!recompute`. It now explicitly checks:

```cpp
covariance_mode == CovarianceMode::NORMALS
```

Reason:

Only the pure normal mode should process the entire cloud using the old
normal-only helper. Hybrid mode must inspect each point separately.

#### Moved the hybrid gate before kNN/PCA

At the beginning of each point iteration:

```cpp
if (covariance_mode == CovarianceMode::HYBRID_NORMAL_QUALITY &&
    HasReliableStoredNormal(query_point)) {
  cov = CovarianceFromStoredNormal(query_point);
  continue;
}
```

Reason:

This ordering is the central behavior of the MVP. A reliable normal avoids the
expensive nearest-neighbour search and SVD. Only rejected normals enter the
existing GICP fallback block.

The earlier experimental implementation performed PCA before making the
decision. That could select between orientations, but it paid the standard
GICP cost for every point and therefore defeated the computational purpose of
the gate.

#### Kept the fallback unchanged

The original kNN covariance calculation and regularized GICP reconstruction are
used for points rejected by the gate.

Reason:

The experiment is intended to test covariance selection, not a newly invented
covariance formula. Keeping the fallback unchanged also provides a clear
comparison with the repository's existing `recompute` behavior.

#### Added atomic path counters

```text
stored_normal
gicp_fallback
```

Reason:

The counters reveal whether the configured threshold is producing a meaningful
mixture of both branches. OpenMP atomic increments are necessary because point
covariances are computed in parallel.

The summary is printed only when timing output is enabled, avoiding additional
normal runtime logging.

#### Passed source and target modes into covariance computation

`computeTransformation` now passes:

```cpp
target_covariance_mode_
source_covariance_mode_
```

instead of the two legacy recomputation booleans.

Reason:

Without this change, the selected hybrid mode would never reach
`computeCovariances`.

### `point_cloud_localization/include/point_cloud_localization/PointCloudLocalization.h`

The localization parameter structure now stores:

```cpp
std::string source_covariance_mode;
std::string target_covariance_mode;
double hybrid_max_curvature;
```

Reason:

Values loaded from ROS parameters must remain available until `SetupICP`
constructs and configures the GICP object.

### `point_cloud_localization/src/PointCloudLocalization.cc`

#### Loaded the new parameters

The following optional parameters are read:

```text
localization/source_covariance_mode
localization/target_covariance_mode
localization/hybrid_max_curvature
```

Defaults are `legacy`, `legacy`, and `0.03`.

Reason:

Optional defaults preserve compatibility with launch files that do not yet
contain the new settings.

#### Applied the settings in `SetupICP`

The original recomputation boolean setters are called first. The string mode
setters are then called to apply an explicit non-legacy override.

Reason:

This order supports both old and new configurations:

- `legacy` retains the result of the original boolean.
- An explicit mode overrides that result.

### `point_cloud_localization/config/parameters.yaml`

### `point_cloud_localization/config/spot_parameters.yaml`

Both localization configurations now use:

```yaml
source_covariance_mode: legacy
target_covariance_mode: hybrid
hybrid_max_curvature: 0.03
```

Reason:

The target is the accumulated local map, where fallback kNN/PCA can use a
richer neighbourhood than the sparse scan that originally produced the stored
normal.

The source remains in legacy mode because recomputing PCA over the same sparse
source scan does not add geometric information.

### `point_cloud_odometry/include/point_cloud_odometry/PointCloudOdometry.h`

The odometry parameter structure received the same three fields as
localization.

Reason:

The GICP class has one common interface, and odometry must be able to select the
same modes when experiments require it.

### `point_cloud_odometry/src/PointCloudOdometry.cc`

#### Loaded parameters from the `icp` namespace

```text
icp/source_covariance_mode
icp/target_covariance_mode
icp/hybrid_max_curvature
```

Reason:

The odometry YAML places its registration parameters under `icp`, not
`localization`.

#### Applied the new settings

After the original recomputation booleans are applied, odometry applies the
source mode, target mode, and curvature threshold.

Reason:

This provides the same backward-compatible override behavior as localization.

### `point_cloud_odometry/config/parameters.yaml`

The defaults are:

```yaml
source_covariance_mode: legacy
target_covariance_mode: legacy
hybrid_max_curvature: 0.03
```

Reason:

Scan-to-scan odometry does not automatically gain a denser fallback
neighbourhood. Leaving both sides in legacy mode avoids changing odometry
behavior before it is evaluated separately.

## 5. Backward Compatibility

Existing configurations continue to work because:

- The original recomputation booleans still exist.
- New mode parameters default to `legacy`.
- `legacy` does not overwrite the mode selected by the original boolean.
- `hybrid_planarity` remains accepted as an alias for `hybrid`.

The new localization YAML explicitly enables hybrid target covariances. The
odometry YAML explicitly retains legacy behavior.

## 6. Intentional Limitations of the MVP

- Curvature alone cannot perfectly distinguish a plane from every degenerate
  local shape, such as a nearly linear neighbourhood.
- The threshold `0.03` is a starting value and requires dataset evaluation.
- The gate depends on curvature being preserved through point-cloud filtering
  and mapping.
- A bad target normal can benefit from PCA over the accumulated target map. A
  bad source normal does not gain new information if fallback PCA uses the same
  source scan.
- Covariances are still cached according to the existing GICP object lifecycle;
  the MVP does not add local-map change tracking or selective invalidation.

These limitations keep the first experiment focused: determine whether a cheap
per-point choice between the existing LOCUS normal covariance and the existing
GICP covariance improves the accuracy/runtime tradeoff.

## 7. Formatting-Only Changes

The diff against `upstream/main` contains extensive indentation and line-wrap
changes in `gicp.h` and `gicp.hpp`. Those changes do not alter behavior.

The functional changes are the mode selection, curvature preservation,
normal-quality gate, fallback routing, parameter wiring, and diagnostics
described above.

## 8. Expected Runtime Diagnostics

Hybrid covariance preparation prints whenever a cloud uses hybrid mode:

```text
Hybrid covariance gate [target]: stored_normal=<count> (<percent>%),
gicp_fallback=<count> (<percent>%), total=<count>, max_curvature=<threshold>
```

Interpretation:

- A fallback count of zero means the threshold accepts every stored normal.
- A stored-normal count of zero means the threshold rejects every stored
  normal, making hybrid mode equivalent to full GICP recomputation.
- A mixture confirms that the per-point gate is active.
- The label, for example `[target]` or `[source]`, shows which cloud was gated.

The branch counts should be recorded together with APE and covariance
computation time when tuning `hybrid_max_curvature`.

The setup step also prints:

```text
Covariance modes: source=<mode>, target=<mode>, hybrid_max_curvature=<threshold>
```

Reason:

The first hybrid experiment produced worse APE and did not show the branch
counts because those counts were tied to timing output. The corrected diagnostic
path makes the experiment observable even when timing logs are disabled, so the
next run can distinguish "hybrid was not enabled", "all points used stored
normals", and "most points fell back to standard GICP".
