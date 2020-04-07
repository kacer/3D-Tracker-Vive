#include "position.h"
#include "Arduino.h"


BaseStation baseStation0 = {
    {-0.462107, -0.199174, 0.864169,
     0.002075, 0.974207, 0.225645,
      -0.886822, 0.106065, -0.449775}, // rotation matrix by rows
    {1.548505, 1.168343, -1.145540}  // origin
};

BaseStation baseStation1 = {
    {-0.741860, 0.160941, -0.650955,
     -0.007757, 0.968645, 0.248326,
      0.670510, 0.189273, -0.717351}, // rotation matrix by rows
    {-0.862957, 1.201664, -1.565771}  // origin
};

bool calcPosition(float32_t *angles, vec3d &position) {
    vec3d ray1, ray2;
    calcRayVector(baseStation0, angles[0], angles[1], ray1);
    calcRayVector(baseStation1, angles[2], angles[3], ray2);

    return intersectLines(baseStation0.origin, ray1, baseStation1.origin, ray2, position);
}

void calcRayVector(BaseStation &baseStation, float32_t angle1, float32_t angle2, vec3d &res) {
    vec3d vertical = {arm_cos_f32(angle1), 0, -arm_sin_f32(angle1)};
    vec3d horizontal = {0, arm_cos_f32(angle2), arm_sin_f32(angle2)};

    vec3d ray = {};
    vectorCrossProduct(horizontal, vertical, ray);
    normalizeVector(ray);

    arm_matrix_instance_f32 stationRotMatrix = {3, 3, baseStation.rotationMatrix};
    arm_matrix_instance_f32 rayVector = {3, 1, ray};
    arm_matrix_instance_f32 rotatedRayVector = {3, 1, res};
    arm_mat_mult_f32(&stationRotMatrix, &rayVector, &rotatedRayVector);
}

bool intersectLines(vec3d &origin1, vec3d &vec1, vec3d &origin2, vec3d &vec2, vec3d &res) {
    vec3d w0 = {};
    arm_sub_f32(origin1, origin2, w0, vec3dSize);

    float32_t a, b, c, d, e;
    arm_dot_prod_f32(vec1, vec1, vec3dSize, &a);
    arm_dot_prod_f32(vec1, vec2, vec3dSize, &b);
    arm_dot_prod_f32(vec2, vec2, vec3dSize, &c);
    arm_dot_prod_f32(vec1, w0, vec3dSize, &d);
    arm_dot_prod_f32(vec2, w0, vec3dSize, &e);

    float32_t denominator = a * c - b * b;
    if(fabsf(denominator) < 1e-5f)
        return false;

    float32_t s = (b * e - c * d) / denominator;
    vec3d ps = {};
    arm_scale_f32(vec1, s, ps, vec3dSize);
    arm_add_f32(ps, origin1, ps, vec3dSize);

    float32_t t = (a * e - b * d) / denominator;
    vec3d pt = {};
    arm_scale_f32(vec2, t, pt, vec3dSize);
    arm_add_f32(pt, origin2, pt, vec3dSize);

    vec3d aux = {};
    arm_add_f32(ps, pt, aux, vec3dSize);
    arm_scale_f32(aux, 0.5f, res, vec3dSize);

    /*float32_t dist;
    arm_sub_f32(ps, pt, aux, vec3dSize);
    dist = vectorLength(aux);
    Serial.println(dist);*/
    
    return true;
}

bool normalizeVector(vec3d &vec) {
    float32_t len = vectorLength(vec);
    if(len == 0.0) {
        return false;
    }

    arm_scale_f32(vec, 1 / len, vec, vec3dSize);

    return true;
}

float32_t vectorLength(vec3d &vec) {
    float32_t pow, res;

    arm_power_f32(vec, vec3dSize, &pow);
    arm_sqrt_f32(pow, &res);

    return res;
}

void vectorCrossProduct(vec3d &vec1, vec3d &vec2, vec3d &res) {
    res[0] = vec1[1]*vec2[2] - vec1[2]*vec2[1];
    res[1] = vec1[2]*vec2[0] - vec1[0]*vec2[2];
    res[2] = vec1[0]*vec2[1] - vec1[1]*vec2[0];
}