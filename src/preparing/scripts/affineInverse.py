 #!/usr/bin/env python
import numpy as np
import math

def affine_transformation_inverse(input_matrix):
    '''
    A = [ M   b  ]
        [ 0   1  ]
    where A is 4x4, M is 3x3, b is 3x1, and the bottom row is (0,0,0,1), then

    inv(A) = [ inv(M)   -inv(M) * b ]
            [   0            1     ]
    '''
    output_matrix = np.array([[0, 0, 0, 0],
                            [0, 0, 0, 0],
                            [0, 0, 0, 0],
                            [0, 0, 0, 1]],
                            dtype=float)
    
    output_matrix[:3, :3] = np.linalg.inv(input_matrix[:3, :3])
    tmp = output_matrix[:3, :3]
    
    b = np.array([[input_matrix[0][3]], 
                    [input_matrix[1][3]], 
                    [input_matrix[2][3]]])
    
    tmp2 = -tmp.dot(b)
    
    output_matrix[0][3] = tmp2[0][0]
    output_matrix[1][3] = tmp2[1][0]
    output_matrix[2][3] = tmp2[2][0]
    
    return output_matrix


if __name__ == '__main__':
    test_matrix = np.array([[1, 0, 0, 10],
                            [0, 1, 0, 20],
                            [0, 0, 1, 30],
                            [0, 0, 0, 1]],
                            dtype=float)

    inv_matrix = affine_transformation_inverse(test_matrix)
    print inv_matrix