import numpy as np



def generate_block_diagonal(rows, columns):
    blkrows = int(rows/columns)
    matrix = np.zeros((rows, 3*columns))
    randBlk = np.random.rand(blkrows,3)
    for i, j in zip(range(0, rows, blkrows), range(0,3*columns,3)):
        matrix[i:i+blkrows, j:j+3] = randBlk
    return matrix

