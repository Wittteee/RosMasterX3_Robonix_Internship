"""Create an OpenCV 4.1-compatible YOLOX ONNX copy.

OpenCV 4.1 converts ONNX integer constants to int32 and cannot import the
opset-11 Resize form or step-2 Slice operators used by YOLOX Focus. This tool
clips sentinel integers, rewrites fixed 2x Resize nodes to legacy Upsample and
removes the fixed Focus subgraph. The C++ runtime performs that exact Focus
operation before inference. The original model is never modified.
"""
from __future__ import annotations

import argparse
import hashlib
from pathlib import Path

import numpy as np
import onnx
from onnx import AttributeProto, TensorProto, helper, numpy_helper


INT32_MIN = np.iinfo(np.int32).min
INT32_MAX = np.iinfo(np.int32).max


def clamp_tensor(tensor: TensorProto) -> int:
    if tensor.data_type != TensorProto.INT64:
        return 0
    values = numpy_helper.to_array(tensor)
    clipped = np.clip(values, INT32_MIN, INT32_MAX)
    changed = int(np.count_nonzero(values != clipped))
    if changed:
        tensor.CopyFrom(
            numpy_helper.from_array(clipped.astype(np.int64), name=tensor.name)
        )
    return changed


def convert(source: Path, destination: Path) -> tuple[int, str]:
    if source.resolve() == destination.resolve():
        raise ValueError("Destination must differ from the original model")
    model = onnx.load(str(source))
    onnx.checker.check_model(model)
    changed = sum(clamp_tensor(tensor) for tensor in model.graph.initializer)
    for node in model.graph.node:
        for attribute in node.attribute:
            if attribute.type == AttributeProto.TENSOR:
                changed += clamp_tensor(attribute.t)
            elif attribute.type == AttributeProto.TENSORS:
                changed += sum(clamp_tensor(tensor) for tensor in attribute.tensors)
            elif attribute.type == AttributeProto.INT:
                if attribute.i < INT32_MIN or attribute.i > INT32_MAX:
                    attribute.i = max(INT32_MIN, min(INT32_MAX, attribute.i))
                    changed += 1
            elif attribute.type == AttributeProto.INTS:
                replacement = [
                    max(INT32_MIN, min(INT32_MAX, value))
                    for value in attribute.ints
                ]
                attribute_changes = sum(
                    a != b for a, b in zip(attribute.ints, replacement)
                )
                changed += attribute_changes
                if attribute_changes:
                    del attribute.ints[:]
                    attribute.ints.extend(replacement)
    initializers = {tensor.name: tensor for tensor in model.graph.initializer}
    for node in model.graph.node:
        if node.op_type != "Resize" or len(node.input) < 3:
            continue
        scales = initializers.get(node.input[2])
        if scales is None:
            raise ValueError("Resize scales are not a constant initializer")
        scale_values = numpy_helper.to_array(scales).reshape(-1).tolist()
        if scale_values != [1.0, 1.0, 2.0, 2.0]:
            raise ValueError(f"Unexpected YOLOX Resize scales: {scale_values!r}")
        mode = next(
            (
                attribute.s.decode("ascii")
                for attribute in node.attribute
                if attribute.name == "mode"
            ),
            "nearest",
        )
        node.op_type = "Upsample"
        del node.input[1:]
        del node.attribute[:]
        node.attribute.extend(
            [
                helper.make_attribute("mode", mode),
                helper.make_attribute("scales", scale_values),
            ]
        )
        changed += 1
    focus_nodes = list(model.graph.node[:9])
    if (
        len(focus_nodes) != 9
        or any(node.op_type != "Slice" for node in focus_nodes[:8])
        or focus_nodes[8].op_type != "Concat"
        or list(focus_nodes[8].output) != ["503"]
        or list(model.graph.node[9].input[:1]) != ["503"]
    ):
        raise ValueError("Unexpected YOLOX Focus graph; refusing compatibility rewrite")
    del model.graph.node[:9]
    model.graph.input[0].CopyFrom(
        helper.make_tensor_value_info(
            "503", TensorProto.FLOAT, [1, 12, 320, 320]
        )
    )
    changed += 9
    used_inputs = {
        input_name
        for node in model.graph.node
        for input_name in node.input
        if input_name
    }
    retained = [
        tensor for tensor in model.graph.initializer if tensor.name in used_inputs
    ]
    changed += len(model.graph.initializer) - len(retained)
    del model.graph.initializer[:]
    model.graph.initializer.extend(retained)
    if changed == 0:
        raise ValueError("Model contains no out-of-int32-range ONNX constants")
    destination.parent.mkdir(parents=True, exist_ok=True)
    onnx.save(model, str(destination))
    digest = hashlib.sha256(destination.read_bytes()).hexdigest()
    return changed, digest


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("destination", type=Path)
    args = parser.parse_args(argv)
    changed, digest = convert(args.source, args.destination)
    print(f"clipped_constants={changed}")
    print(f"sha256={digest}")
    print(args.destination)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
