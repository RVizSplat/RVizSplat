"""Minimal PLY vertex reader for 3DGS-format files.

Returns a numpy structured array of the vertex element's scalar properties.
Supports binary_little_endian and ASCII formats; list properties and non-vertex
elements are skipped. No external deps beyond numpy — avoids the plyfile
dependency, which has no rosdep rule and therefore isn't buildfarm-supported.
"""

import numpy as np


# Little-endian field codes matching PLY scalar property types. Forcing '<'
# keeps binary parsing correct on big-endian hosts too.
_PLY_TYPE_TO_NP_LE = {
    'float':   '<f4', 'float32': '<f4',
    'double':  '<f8', 'float64': '<f8',
    'char':    '<i1', 'int8':    '<i1',
    'uchar':   '<u1', 'uint8':   '<u1',
    'short':   '<i2', 'int16':   '<i2',
    'ushort':  '<u2', 'uint16':  '<u2',
    'int':     '<i4', 'int32':   '<i4',
    'uint':    '<u4', 'uint32':  '<u4',
}


def read_ply_vertex(path: str) -> np.ndarray:
    """Return the vertex element of *path* as a numpy structured array.

    Raises ValueError on malformed or unsupported files.
    """
    with open(path, 'rb') as f:
        if f.readline().strip() != b'ply':
            raise ValueError(f'Not a PLY file: {path}')

        fmt = None
        vertex_count = 0
        in_vertex = False
        props: list[tuple[str, str]] = []  # (name, ply_type) in declaration order

        while True:
            line = f.readline()
            if not line:
                raise ValueError('Unexpected end of PLY header')
            tokens = line.strip().decode('ascii', errors='replace').split()
            if not tokens:
                continue
            head = tokens[0]
            if head == 'end_header':
                break
            if head == 'format':
                if tokens[1] == 'binary_little_endian':
                    fmt = 'binary_le'
                elif tokens[1] == 'ascii':
                    fmt = 'ascii'
                else:
                    raise ValueError(f'Unsupported PLY format: {tokens[1]}')
            elif head == 'element':
                in_vertex = (tokens[1] == 'vertex')
                if in_vertex:
                    vertex_count = int(tokens[2])
            elif head == 'property' and in_vertex:
                if tokens[1] == 'list':
                    continue  # list properties aren't used by 3DGS files
                ty, name = tokens[1], tokens[2]
                if ty not in _PLY_TYPE_TO_NP_LE:
                    raise ValueError(f'Unsupported PLY property type: {ty}')
                props.append((name, ty))

        if fmt is None:
            raise ValueError('PLY format not specified')
        if vertex_count <= 0:
            raise ValueError('PLY vertex count is 0 or missing')
        if not props:
            raise ValueError('PLY has no vertex properties')

        dtype = np.dtype([(n, _PLY_TYPE_TO_NP_LE[t]) for (n, t) in props])

        if fmt == 'binary_le':
            need = dtype.itemsize * vertex_count
            buf = f.read(need)
            if len(buf) < need:
                raise ValueError('Unexpected end of PLY binary data')
            return np.frombuffer(buf, dtype=dtype, count=vertex_count).copy()

        # ASCII: one vertex per whitespace-separated line.
        arr = np.zeros(vertex_count, dtype=dtype)
        names = dtype.names
        for i in range(vertex_count):
            line = f.readline()
            if not line:
                raise ValueError(
                    f'Unexpected end of ASCII PLY data at vertex {i}')
            toks = line.strip().decode('ascii', errors='replace').split()
            if len(toks) < len(names):
                raise ValueError(
                    f'Too few values at ASCII vertex {i}')
            for j, name in enumerate(names):
                arr[name][i] = float(toks[j])
        return arr
