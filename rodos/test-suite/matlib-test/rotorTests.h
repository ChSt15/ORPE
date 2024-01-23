//Tests the functions of rotor

#define FAIL {PRINTF("FAILED at line %d in file %s\n", __LINE__, __FILE__); failed++;}; 

#define NUMBER_OF_TESTS 100

int rotorTests() {
    int failed = 0;
    
    AngleAxis rot;
    Vector3D start, end;
    
    for (int i = 0; i < NUMBER_OF_TESTS; i++) {
        start.x = drand(RANGE);
        start.y = drand(RANGE);
        start.z = drand(RANGE);
        
        AngleAxis aRot(abs(drand() * 2 * M_PI), Vector3D(drand(RANGE), drand(RANGE), drand(RANGE)));
        end = start.aRotate(aRot);
        
        rot = AngleAxis::rotor(start, end);
        if (!end.equals(start.qRotate(Quaternion(rot)))) FAIL;
    }
    
    //isNoRotation
    rot.u.x = 1;
    rot.u.y = 2;
    rot.u.z = 3;
    rot.phi = 1;
    if (!rot.isNoRotation()) FAIL;
    
    rot.phi = 0.5;
    if (rot.isNoRotation()) FAIL;
    
    rot.phi = 1.5;
    if (!rot.isNoRotation()) FAIL;

    rot.phi = -1.5;
    if (!rot.isNoRotation()) FAIL;
    
    rot.u.x = 0;
    rot.u.y = 0;
    rot.u.z = 0;
    rot.phi = 0.5;
    if (!rot.isNoRotation()) FAIL;
    
    //ifNaN
    rot.u.x = 1;
    rot.u.y = 2;
    rot.u.z = 3;
    rot.phi = 1;
    if (rot.resetIfNAN()) FAIL;
    
    rot.u.x = static_cast<double>(NAN);
    if (!rot.resetIfNAN()) FAIL;
    if (rot.phi != 0) FAIL;
    
    rot.u.y = static_cast<double>(NAN);
    if (!rot.resetIfNAN()) FAIL;
    rot.u.z = static_cast<double>(NAN);
    if (!rot.resetIfNAN()) FAIL;
    rot.phi = static_cast<double>(NAN);
    if (!rot.resetIfNAN()) FAIL;
    
    return failed;
}
