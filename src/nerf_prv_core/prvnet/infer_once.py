import os
import sys
import types
import collections.abc


MIN_LABEL_VALUE = 13
MAX_LABEL_VALUE = 58


def read_config_value(file_path, key):
    with open(file_path, "r", encoding="utf-8") as cfg:
        for raw_line in cfg:
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            if line.startswith(f"{key}:"):
                value = line.split(":", 1)[1].strip()
                if value.startswith('"') and value.endswith('"'):
                    value = value[1:-1]
                return value
    return None


def write_budget(output_dir, budget):
    os.makedirs(output_dir, exist_ok=True)
    with open(os.path.join(output_dir, "view_budget.txt"), "w", encoding="utf-8") as fout:
        fout.write(f"{int(budget)}\n")


def list_image_paths(images_dir):
    if not os.path.isdir(images_dir):
        return []

    def image_key(name):
        stem = os.path.splitext(name)[0]
        return (0, int(stem)) if stem.isdigit() else (1, stem)

    image_names = [
        name
        for name in os.listdir(images_dir)
        if os.path.isfile(os.path.join(images_dir, name)) and name.lower().endswith(".png")
    ]
    image_names.sort(key=image_key)
    return [os.path.join(images_dir, name) for name in image_names]


def try_real_prvnet(prvnet_checkpoint, prvnet_env_path, image_paths):
    if not image_paths:
        raise RuntimeError("no RGB images found for PRVNet inference")

    if prvnet_env_path not in sys.path:
        sys.path.insert(0, prvnet_env_path)

    if "torch._six" not in sys.modules:
        torch_six = types.ModuleType("torch._six")
        torch_six.container_abcs = collections.abc
        torch_six.string_classes = (str, bytes)
        torch_six.int_classes = (int,)
        sys.modules["torch._six"] = torch_six

    if "MinkowskiEngine" not in sys.modules:
        minkowski_engine = types.ModuleType("MinkowskiEngine")

        class SparseTensor:  # pragma: no cover - compatibility shim for import only
            pass

        minkowski_engine.SparseTensor = SparseTensor
        sys.modules["MinkowskiEngine"] = minkowski_engine

    import torch
    import torchvision.transforms as transforms
    import timm
    from PIL import Image
    import models.convnextv2 as convnextv2

    class PVBNet(torch.nn.Module):
        def __init__(self, convnext_model, fc_dim=None):
            super().__init__()
            if fc_dim is None:
                fc_dim = [1000, 500, 250, 100, 1]
            self.encoder = convnext_model
            self.fc_layer = torch.nn.Sequential(
                torch.nn.Linear(fc_dim[0] * 2, fc_dim[0]),
                torch.nn.Linear(fc_dim[0], fc_dim[1]),
                torch.nn.Linear(fc_dim[1], fc_dim[2]),
                torch.nn.Linear(fc_dim[2], fc_dim[3]),
                torch.nn.Linear(fc_dim[3], fc_dim[4]),
            )

        def forward(self, xs):
            outputs = []
            for tensor in xs:
                outputs.append(self.encoder(tensor))
            outputs = torch.stack(outputs)
            mean = torch.mean(outputs, dim=0)
            variance = torch.var(outputs, dim=0)
            fused = torch.cat([mean, variance], dim=-1)
            return self.fc_layer(fused)

    assert timm.__version__ == "0.3.2"

    transform = transforms.Compose(
        [
            transforms.CenterCrop(size=720),
            transforms.ToTensor(),
        ]
    )

    device = "cpu"
    encoder = convnextv2.__dict__["convnextv2_tiny"](
        num_classes=1000,
        drop_path_rate=0.0,
        head_init_scale=0.001,
    )
    model = PVBNet(convnext_model=encoder)

    checkpoint = torch.load(prvnet_checkpoint, map_location=device)
    state_dict = checkpoint.get("model_state_dict", checkpoint)
    normalized_state_dict = {}
    for key, value in state_dict.items():
        normalized_key = key
        if normalized_key.startswith("module."):
            normalized_key = normalized_key[len("module."):]
        normalized_state_dict[normalized_key] = value
    model.load_state_dict(normalized_state_dict, strict=True)
    model.to(device)
    model.eval()

    tensors = []
    for image_path in image_paths:
        image = Image.open(image_path).convert("RGB")
        tensors.append(transform(image).unsqueeze(0).to(device))

    if len(tensors) == 1:
        tensors.append(tensors[0].clone())

    with torch.no_grad():
        pred = model(tensors)
        pred = torch.nn.functional.sigmoid(pred)
        pred = MIN_LABEL_VALUE + (MAX_LABEL_VALUE - MIN_LABEL_VALUE) * pred
        pred = torch.round(pred)

    return int(pred.item())


if __name__ == "__main__":
    config_path = sys.argv[1] if len(sys.argv) > 1 else None
    if not config_path:
        work_dir = os.environ.get("WORK_DIR", "")
        config_path = os.path.join(work_dir, "src", "nerf_prv_core", "config", "DefaultConfiguration.yaml")

    data_dir = read_config_value(config_path, "prvnet_data_dir") or ""
    external_root = read_config_value(config_path, "external_root") or ""
    fallback_budget = int(read_config_value(config_path, "fallback_view_budget") or 12)
    use_prvnet_prediction = int(read_config_value(config_path, "use_prvnet_prediction") or 0)
    prvnet_checkpoint = read_config_value(config_path, "prvnet_checkpoint") or ""
    prvnet_env_path = read_config_value(config_path, "prvnet_env_path") or ""

    if not prvnet_checkpoint and external_root:
        prvnet_checkpoint = os.path.join(external_root, "prvnet", "checkpoints", "best_checkpoint.pth")
    if not prvnet_env_path and external_root:
        prvnet_env_path = os.path.join(external_root, "convnext_v2")

    image_paths = list_image_paths(os.path.join(data_dir, "images"))

    budget = fallback_budget
    reason = "fallback budget"

    if (
        use_prvnet_prediction
        and os.path.isfile(prvnet_checkpoint)
        and os.path.isdir(prvnet_env_path)
    ):
        try:
            budget = try_real_prvnet(prvnet_checkpoint, prvnet_env_path, image_paths)
            reason = "real PRVNet inference"
        except Exception as exc:
            reason = f"fallback after PRVNet error: {exc}"
    elif image_paths:
        budget = max(fallback_budget, len(image_paths))
        reason = "image-aware fallback"

    write_budget(data_dir, budget)
    print(f"nerf_prv_core.prvnet: wrote budget {budget} ({reason})")
