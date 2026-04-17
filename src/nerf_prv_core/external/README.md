## External Assets Layout

This directory mirrors the assets that the public `psc0628/NeRF-PRV` repository
expects but does not ship.

Place external files here when you want to move from the local fallback
benchmark to the original PRVNet or full NeRF-PRV pipeline.

Expected layout:

- `prvnet/checkpoints/best_checkpoint.pth`
- `prvnet/imagenet/convnextv2_tiny_1k_224_ema.pt`
- `prvnet/data_5view/`
- `convnext_v2/`
- `shapenet/`
- `instant_ngp/scripts/`
- `origin_exports/`

Notes:

- `best_checkpoint.pth` and `data_5view/` come from the Kaggle dataset linked in
  `upstream/PRVNet/README.md`.
- `convnext_v2/` is the external ConvNeXt-V2 codebase that upstream
  `infer_server.py` imports from.
- `instant_ngp/scripts/` is the external instant-ngp installation target that
  upstream `Instantngp_scripts/README.md` refers to.
- `shapenet/` is for `ShapeNetCore.v2`.
