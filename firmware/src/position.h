#include <arm_math.h>

const uint8_t vec3dSize = 3;
typedef float32_t vec3d[vec3dSize];

typedef struct {
    float32_t rotationMatrix[9];
    vec3d origin;
} BaseStation;

/**
 * @brief Function will compute position of tracked object from given angles.
 * 
 * @param angles of base stations [M-A0, M-A1, S-A0, S-A1]
 * @param position calculated position (x, y, z)
 * @return true if the computation was successful
 * @return false otherwise
 */
bool calcPosition(float32_t *angles, vec3d &position);

/**
 * @brief Function will compute ray vector from base station to sensor.
 * 
 * @param baseStation struct with base station origin and rotation matrix
 * @param angle1 vertical angle
 * @param angle2 horizontal angle
 * @param res result vector
 */
void calcRayVector(BaseStation &baseStation, float32_t angle1, float32_t angle2, vec3d &res);

/**
 * @brief Function will find intersection between two vectors from base stations 
 * and therefore compute 3D position of tracked object per this algorithm:
 * http://geomalgorithms.com/a07-_distance.html#Distance-between-Lines. 
 * 
 * @param origin1 origin of base station 0 (Master)
 * @param vec1 vector from base station 0 to sensor of tracked object
 * @param origin2 origin of base station 1 (Slave)
 * @param vec2 vector from base station 1 to sensor of tracked object
 * @param res coordinates (x, y, z) of tracked object
 * @return true if computation was successful
 * @return false otherwise
 */
bool intersectLines(vec3d &origin1, vec3d &vec1, vec3d &origin2, vec3d &vec2, vec3d &res);

/**
 * @brief Function will try to normalize given vector.
 * 
 * @param vec vector
 * @return true if vector was normalized
 * @return false otherwise
 */
bool normalizeVector(vec3d &vec);

/**
 * @brief Compute length of given vector.
 * 
 * @param vec vector
 * @return float32_t length of vector
 */
float32_t vectorLength(vec3d &vec);

/**
 * @brief Function will compute cross product of two given vectors.
 * 
 * @param vec1 vector 1
 * @param vec2 vector 2
 * @param res cross product of vec1 and vec2
 */
void vectorCrossProduct(vec3d &vec1, vec3d &vec2, vec3d &res);
