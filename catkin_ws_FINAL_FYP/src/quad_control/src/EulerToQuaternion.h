#ifdef euler_to_quaternion_h
#define euler_to_quaternion_h

using namespace std;

void e2q(float *euler_ang, float *y);

void q2e(float *quat, float *euler);

#endif // !euler_to_quaternion_h