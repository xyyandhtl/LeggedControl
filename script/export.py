#!/usr/bin/env python3
import argparse
import torch

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--jit", type=str, default="/home/lenovo/Projects/himloco/legged_gym/logs/stairs_aliengo/amp_0806/exported/policy.pt", help="input TorchScript .pt")
    p.add_argument("--onnx", type=str, default="policy.onnx", help="output ONNX path")
    p.add_argument("--obs-dim", type=int, default=270, help="observation dimension (per-frame)")
    p.add_argument("--opset", type=int, default=14, help="ONNX opset version")
    args = p.parse_args()

    # 加载 TorchScript（Script/Trace 均可）
    print(f"Loading TorchScript: {args.jit}")
    model = torch.jit.load(args.jit, map_location="cpu")
    model.eval()

    # 构造占位输入：形状 [1, obs_dim]
    dummy = torch.zeros(1, args.obs_dim, dtype=torch.float32)

    # 先跑一遍，确保可导出
    with torch.no_grad():
        res = model(dummy)
        print(res)

    print(f"Exporting to ONNX: {args.onnx}")
    torch.onnx.export(
        model,
        (dummy,),
        args.onnx,
        export_params=True,
        opset_version=args.opset,
        do_constant_folding=True,
        input_names=["obs"],
        output_names=["action"],
        dynamic_axes={"obs": {0: "batch"}, "action": {0: "batch"}},
    )
    print("Done.")

if __name__ == "__main__":
    main()