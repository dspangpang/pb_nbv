# NeRF-PRV Local Hook

This folder provides the local one-shot budget prediction hook used by the
current `nerf_prv_core` integration.

Current behavior:

- `infer_once.py` is runnable in this repository as-is.
- It writes a fallback or image-aware budget so the benchmark can run even
  without the full upstream PRVNet stack.

Upstream repo content now vendored locally:

- `src/nerf_prv_core/upstream/PRVNet/`
- `src/nerf_prv_core/upstream/Instantngp_scripts/`
- `src/nerf_prv_core/upstream/ShapeNet_scripts/`
- `src/nerf_prv_core/upstream/Origin_scripts/`
- `src/nerf_prv_core/upstream/PRV_simulation_reference/`

Prepared local external-data layout:

- `src/nerf_prv_core/external/prvnet/checkpoints/`
- `src/nerf_prv_core/external/prvnet/imagenet/`
- `src/nerf_prv_core/external/prvnet/data_5view/`
- `src/nerf_prv_core/external/convnext_v2/`
- `src/nerf_prv_core/external/shapenet/`
- `src/nerf_prv_core/external/instant_ngp/scripts/`

What is still external:

- ConvNeXt-V2 codebase required by upstream `infer_server.py`
- PRVNet checkpoint
- ImageNet pretrained ConvNeXt-V2 weights
- PRV dataset
- instant-ngp installation

See:

- `src/nerf_prv_core/EXTERNAL_DATA.md`
