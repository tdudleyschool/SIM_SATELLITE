/*
Purpose:    (Place where all record definitions go. These records will 
            be used for defining necessary datatypes that can be shared 
            by multiple classes in diffrent files.)
*/


#ifdef __cplusplus
    extern "C"
    {
#endif

struct Vector3d_pos  {
    double v3[3];
    double p3[3];
};

#ifdef __cplusplus
    }
#endif