# NeRF-PRV External Data Checklist

The local workspace now contains:

- all files that are actually present in the public
  `psc0628/NeRF-PRV` GitHub repository
- a local `external/` skeleton that matches the assets required by the upstream
  READMEs but not shipped in git

Vendored from upstream:

- `upstream/PRVNet/`
- `upstream/Instantngp_scripts/`
- `upstream/ShapeNet_scripts/`
- `upstream/Origin_scripts/`
- `upstream/PRV_simulation_reference/`
- `view_space/Hemisphere/`

Prepared local drop-in targets:

- `external/prvnet/checkpoints/`
- `external/prvnet/imagenet/`
- `external/prvnet/data_5view/`
- `external/convnext_v2/`
- `external/shapenet/`
- `external/instant_ngp/scripts/`
- `external/origin_exports/`

What is still external and must be downloaded manually:

## 1. PRVNet checkpoint

Upstream source:
- `PRVNet/README.md` says to download `best_checkpoint.pth` from the Kaggle
  release.

Local target:
- `src/nerf_prv_core/external/prvnet/checkpoints/best_checkpoint.pth`

## 2. ConvNeXt-V2 pretrained ImageNet model

Upstream source:
- `PRVNet/README.md` says to download
  `convnextv2_tiny_1k_224_ema.pt`.

Local target:
- `src/nerf_prv_core/external/prvnet/imagenet/convnextv2_tiny_1k_224_ema.pt`

## 3. ConvNeXt-V2 codebase

Upstream source:
- `PRVNet/infer_server.py` and `train_regression.py` import
  `engine_pretrain`, `models.convnextv2`, `models.fcmae`, and `utils`, which
  are not shipped in `NeRF-PRV` itself.

Local target:
- `src/nerf_prv_core/external/convnext_v2/`

## 4. PRV dataset

Upstream source:
- `PRVNet/README.md` points to the Kaggle PRV dataset.

Local target:
- `src/nerf_prv_core/external/prvnet/data_5view/`

## 5. ShapeNetCore.v2

Upstream source:
- `ShapeNet_scripts/README.md` requires ShapeNetCore.v2.

Local target:
- `src/nerf_prv_core/external/shapenet/`

## 6. instant-ngp

Upstream source:
- `Instantngp_scripts/README.md` requires an external `instant-ngp`
  installation.

Local target:
- `src/nerf_prv_core/external/instant_ngp/scripts/`

## 7. OriginPro

Upstream source:
- `Origin_scripts/README.md` requires OriginPro for curve fitting.

Local target:
- `src/nerf_prv_core/external/origin_exports/`
