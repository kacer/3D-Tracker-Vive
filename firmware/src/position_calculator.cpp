#include "position_calculator.h"
#include "Arduino.h"

int PositionCalculator::calcPosition(float32_t *angles, vec3d &position) {
    // check whether base stations geometry is set
    for(uint8_t b = 0; b < baseStationsCount; b++) {
        if(!baseStations[b].isSet()) {
            return BASESTATIONS_NOT_SET;
        }
    }

    vec3d ray1, ray2;
    calcRayVector(baseStations[0], angles[0], angles[1], ray1);
    calcRayVector(baseStations[1], angles[2], angles[3], ray2);

    return intersectLines(baseStations[0].origin, ray1, baseStations[1].origin, ray2, position);
}

void PositionCalculator::setBaseStationsGeometry(BaseStation b0, BaseStation b1) {
    memcpy(&baseStations[0], &b0, sizeof(BaseStation));
    memcpy(&baseStations[1], &b1, sizeof(BaseStation));
}

void PositionCalculator::calcRayVector(BaseStation &baseStation, float32_t angle1, float32_t angle2, vec3d &res) {
    vec3d vertical = {0, arm_cos_f32(angle2), arm_sin_f32(angle2)};
    vec3d horizontal = {arm_cos_f32(angle1), 0, -arm_sin_f32(angle1)};

    vec3d ray = {};
    vectorCrossProduct(vertical, horizontal, ray);
    normalizeVector(ray);

    arm_matrix_instance_f32 stationRotMatrix = {3, 3, baseStation.rotationMatrix};
    arm_matrix_instance_f32 rayVector = {3, 1, ray};
    arm_matrix_instance_f32 rotatedRayVector = {3, 1, res};
    arm_mat_mult_f32(&stationRotMatrix, &rayVector, &rotatedRayVector);
}

int PositionCalculator::intersectLines(vec3d &origin1, vec3d &vec1, vec3d &origin2, vec3d &vec2, vec3d &res) {
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
        return CALCULATION_FAILED;

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
    
    return CALCULATION_SUCCESS;
}

bool PositionCalculator::normalizeVector(vec3d &vec) {
    float32_t len = vectorLength(vec);
    if(len == 0.0) {
        return false;
    }

    arm_scale_f32(vec, 1 / len, vec, vec3dSize);

    return true;
}

float32_t PositionCalculator::vectorLength(vec3d &vec) {
    float32_t pow, res;

    arm_power_f32(vec, vec3dSize, &pow);
    arm_sqrt_f32(pow, &res);

    return res;
}

void PositionCalculator::vectorCrossProduct(vec3d &vec1, vec3d &vec2, vec3d &res) {
    res[0] = vec1[1]*vec2[2] - vec1[2]*vec2[1];
    res[1] = vec1[2]*vec2[0] - vec1[0]*vec2[2];
    res[2] = vec1[0]*vec2[1] - vec1[1]*vec2[0];
}