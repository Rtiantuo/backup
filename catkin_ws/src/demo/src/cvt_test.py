import tf
import numpy as np

t_body = np.array([[1.37116732],[-0.1030159],[0.]])
rol,pit = 0,0
yaw = np.pi/2

if __name__ == '__main__':
    q_ = tf.transformations.quaternion_from_euler(rol,pit,yaw,'sxyz')
    M_ = tf.transformations.quaternion_matrix(q_)
    t_map = np.matmul(M_[:3,:3],t_body)
    print t_map