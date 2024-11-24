import os
import numpy as np

def read_vectors(file_path):
    with open(file_path, 'r') as file:
        lines = file.readlines()
    vectors = [list(map(float, line.strip().split())) for line in lines]
    return vectors

def extend_vectors(vectors, scale_factor):
    extended_vectors = [np.abs(np.array(vector) * scale_factor) for vector in vectors]
    return extended_vectors

def write_vectors(file_path, vectors):
    with open(file_path, 'w') as file:
        for vector in vectors:
            file.write(' '.join(map(str, vector)) + '\n')

if __name__ == "__main__":
    current_dir = os.path.dirname(os.path.abspath(__file__))
    input_file_path = os.path.join(current_dir, 'view_space.txt')
    output_file_path = os.path.join(current_dir, 'extended_view_space.txt')
    scale_factor = 3

    # 读取向量
    vectors = read_vectors(input_file_path)
    print("Original Vectors:")
    for vector in vectors:
        print(vector)

    # 扩展向量并计算绝对值
    extended_vectors = extend_vectors(vectors, scale_factor)
    print("\nExtended Vectors (with absolute values):")
    for vector in extended_vectors:
        vector_norm = np.linalg.norm(vector)
        print(vector_norm)

    # 写入扩展后的向量
    write_vectors(output_file_path, extended_vectors)
    print(f"\nExtended vectors written to {output_file_path}")