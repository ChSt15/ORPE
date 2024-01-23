//Tests for 6D Matrices
#define FAIL {PRINTF("FAILED at line %d in file %s\n", __LINE__, __FILE__); failed++;};

#define NUMBER_OF_TESTS 100
#define RANGE           5

int matrix6DTests() {
    int failed = 0;
    
    Matrix6D m1, m2, m3, m4;
    Vector6D v6d, v6d2;
    Matrix3D m3d1, m3d2;
    
    double rnd[36], r, r2[6];
    
    for (size_t i = 0; i < NUMBER_OF_TESTS; i++) {
        for (size_t j = 0; j < 36; j++) {
            rnd[j] = drand(RANGE);
        }
        m1 = Matrix6D(rnd);
        for (size_t j = 0; j < 36; j++) {
            rnd[j] = drand(RANGE);
        }
        m2 = Matrix6D(rnd);
        
        // + and -
        m3 = m1 + m2;
        for (size_t j = 0; j < 6; j++) {
            for (size_t k = 0; k < 6; k++) {
                m4.r[j][k] = m1.r[j][k] + m2.r[j][k];
            }
        }
        if (m3 != m4) FAIL;
        
        m3 = m1 - m2;
        for (size_t j = 0; j < 6; j++) {
            for (size_t k = 0; k < 6; k++) {
                m4.r[j][k] = m1.r[j][k] - m2.r[j][k];
            }
        }
        if (m3 != m4) FAIL;
        
        // /
        r = drand(RANGE);
        m3 = m1 / r;
        for (size_t j = 0; j < 6; j++) {
            for (size_t k = 0; k < 6; k++) {
                m4.r[j][k] = m1.r[j][k] / r;
            }
        }
        if (m3 != m4) FAIL;
        
        // * double
        m3 = m1 * r;
        for (size_t j = 0; j < 6; j++) {
            for (size_t k = 0; k < 6; k++) {
                m4.r[j][k] = m1.r[j][k] * r;
            }
        }
        if (m3 != m4) FAIL;
        
        // * Matrix6D
        m3 = m1 * m2;
        m4 = m4 - m4; //Sets all elements to 0
        for (size_t j = 0; j < 6; j++) {
            for (size_t k = 0; k < 6; k++) {
                for (size_t l = 0; l < 6; l++) {
                    m4.r[j][k] += m1.r[j][l] * m2.r[l][k];
                }
            }
        }
        if (!m3.equals(m4)) FAIL;
        
        //invert
        m3 = m1.invert();
        m4 = Matrix6D::eye(1);
        m3 = m3 * m1;
        if (m3 != m4) FAIL;
        
        m3 = m1.invert();
        m4 = Matrix6D::eye(1);
        m3 = m1 * m3;
        if (m3 != m4) FAIL;
        
        //Dyadic product
        for (size_t j = 0; j < 6; j++) {
            r2[j] = drand(RANGE);
        }
        v6d = Vector6D(r2);
        
        for (size_t j = 0; j < 6; j++) {
            r2[j] = drand(RANGE);
        }
        v6d2 = Vector6D(r2);
        
        m3 = v6d * v6d2.transpose();	// transpose is required, otherwise dot product!
        for (size_t j = 0; j < 6; j++) {
            for (size_t k = 0; k < 6; k++) {
                m4.r[j][k] = v6d.v[j] * v6d2.v[k];
            }
        }
        if (m3 != m4) FAIL;
        
        //Transpose
        m3 = m1.transpose();
        for (size_t j = 0; j < 6; j++) {
            for (size_t k = 0; k < 6; k++) {
                m4.r[j][k] = m1.r[k][j];
            }
        }
        
        //Access
        m3d1 = m1.upperLeft();
        for (size_t j = 0; j < 3; j++) {
            for (size_t k = 0; k < 3; k++) {
                m3d2.r[j][k] = m1.r[j][k];
            }
        }
        if (m3d1 != m3d2) FAIL;
        
        m3d1 = m1.upperRight();
        for (size_t j = 0; j < 3; j++) {
            for (size_t k = 0; k < 3; k++) {
                m3d2.r[j][k] = m1.r[j][k+3];
            }
        }
        if (m3d1 != m3d2) FAIL;
        
        m3d1 = m1.lowerLeft();
        for (size_t j = 0; j < 3; j++) {
            for (size_t k = 0; k < 3; k++) {
                m3d2.r[j][k] = m1.r[j+3][k];
            }
        }
        if (m3d1 != m3d2) FAIL;
        
        m3d1 = m1.lowerRight();
        for (size_t j = 0; j < 3; j++) {
            for (size_t k = 0; k < 3; k++) {
                m3d2.r[j][k] = m1.r[j+3][k+3];
            }
        }
        if (m3d1 != m3d2) FAIL;
        
        //Rows
        for (size_t j = 0; j < 6; j++) {
            m1.setRow(j, v6d.transpose());		// transpose is required, otherwise v6d is a column
            v6d2 = m1.getRow(j).transpose();	// transpose is required, transform row to column
            if (v6d != v6d2) FAIL;
        }
        
        //Columns
        for (size_t j = 0; j < 6; j++) {
            m1.setColumn(j, Matrix_<6, 1, double>(v6d)); // ToDo avoid constructor call
            v6d2 = m1.getColumn(j);
            if (v6d != v6d2) FAIL;
        }
    }
    
    return failed;
}
 
