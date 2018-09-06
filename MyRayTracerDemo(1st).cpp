#include <iostream>
#include <vector>
#include <fstream>
#include <cassert>
#include <cstdlib>
#include <cstdio>
#include <cmath>

#define M_PI 3.141592653589793
#define INFINITY 1e8

//�Զ���������
template<typename T>
class Vec3
{
public:
    T x,y,z;
    //��ʼ��
    Vec3():x(T(0)),y(T(0)),z(T(0)){}
    Vec3(T xx):x(xx),y(xx),z(xx){}
    Vec3(T xx,T yy,T zz):x(xx),y(yy),z(zz){}
    Vec3& normalize() //��һ��
    {
		T nor2 = length2();//���¶���
        if(nor2>0)
        {
			T invNor = 1 / sqrt(nor2);
			x *= invNor, y *= invNor, z *= invNor;
        }
        return *this;
    }
	Vec3<T> operator * (const T &f) const { return Vec3<T>(x*f, y*f, z*f); } //��һ������
	Vec3<T> operator * (const Vec3<T> &v) const { return Vec3<T>(x*v.x, y*v.y, z*v.z); } //��һ������
	Vec3<T> operator - (const Vec3<T> &v) const { return Vec3<T>(x - v.x, y - v.y, z - v.z); } //��һ������
	Vec3<T> operator + (const Vec3<T> &v) const { return Vec3<T>(x + v.x, y + v.y, z + v.z); } //��һ������
	Vec3<T>& operator += (const Vec3<T> &v) { x += v.x, y += v.y, z += v.z; return *this; }
	Vec3<T>& operator *= (const Vec3<T> &v)  { x *= v.x, y *= v.y, z *= v.z; return *this; }
	Vec3<T> operator - () const { return Vec3<T>(-x, -y, -z); } //ȡ��
    T dot(const Vec3<T> &v) const {return x * v.x + y * v.y + z * v.z;} //�������,��˵ļ��κ����ҾͲ���˵��
	T length2() const { return x * x + y * y + z * z; } //ģ^2
	T length() const { return sqrt(length2()); } //ģ

    friend std::ostream & operator << (std::ostream &os,const Vec3<T> &v)
    {
        os << "[" <<v.x<<" "<<v.y<<" "<<v.z<<"]";
        return os;
    }
};

typedef Vec3<float> Vec3f;

//�Զ���������

class Sphere
{
public:
    Vec3f center; //�����λ��
    float radius,radius2; //����İ뾶�Ͱ뾶��ƽ��
    Vec3f surfaceColor,emissionColor; //������ɫ�ͷ�ɢ��ɫ
    float transparency,reflection; //����͸���Ⱥͷ�����
    Sphere(
        const Vec3f &c,const float &r,
        const Vec3f &sc,const float &refl=0,
        const float &transp=0,const Vec3f &ec=0):
        center(c),radius(r),radius2(r*r),surfaceColor(sc),emissionColor(ec),
        transparency(transp),reflection(refl)
        {}

    //���������ཻ����һ��Ҫ�ú����
    bool intersect(const Vec3f &rayorig,const Vec3f &raydir,float &t0,float &t1) const
    {
        Vec3f l=center-rayorig; 
        float tca=l.dot(raydir); 
        if (tca<0) return false; 
        float d2=l.dot(l)-tca * tca; 
        if (d2>radius2) return false; 
        float thc=sqrt(radius2-d2); 
        t0=tca-thc; 
        t1=tca+thc; 
 
        return true; 
		
    }
};


#define MAX_RAY_DEPTH 5 //���ݹ����

float mix(const float &a,const float &b,const float &mix)
{
	return b * mix + a * (1 - mix);
}

/*��һ������Ҫ��׷�ٺ�����ȡһ��������Ϊ������������ԭ��ͷ����壩��������������
�Ƿ��볡���е��κμ���ͼ���ཻ�����������һ�������ཻ��������ཻ�㡢�ཻ���
��������������Щ��Ϣ��ɫ����㡣��ɫȡ���������������ԣ��Ƿ�͸�������⡢�����䣩��
��������뽻��㴦������ɫ�Ķ����ཻ����ú������ع��ߵ���ɫ�����򷵻ر�����ɫ��*/
Vec3f trace(const Vec3f &rayorig,const Vec3f &raydir,
            const std::vector<Sphere> &spheres,const int &depth)
{
    float tnear=INFINITY;
    const Sphere* sphere=NULL;
    //�ҵ������볡���е�������ཻ��
    for(unsigned i=0;i<spheres.size();i++)
    {
        float t0=INFINITY,t1=INFINITY;
        if(spheres[i].intersect(rayorig,raydir,t0,t1))
        {
            if(t0<0) t0=t1;
            if(t0<tnear)
            {
                tnear=t0;
                sphere=&spheres[i];
            }
        }
    }
    //���û���ཻ���򷵻غ�ɫ���߱���ɫ
    if(!sphere) return Vec3f(2);
	Vec3f surfaceColor = 0; //���ߡ�������ཻ������������ɫ
	Vec3f phit = rayorig + raydir * tnear; //�ཻ��
	Vec3f nhit = phit - sphere->center;//�ཻ��ķ�����
    nhit.normalize(); //��һ��������
    //����������͹۲췽�����໥�෴�ģ���ת�������ķ���
    //��Ҳ��ζ�������������ڲ����������ò��������Ƿ����ڲ���
    //Ϊ���桱�����գ���תIdotN�ı��
	float bias = 1e-1; //����һЩƫб�����ǽ�׷�ٵĵ�
	bool inside = false;
	if (raydir.dot(nhit) > 0)
		nhit = -nhit, inside = true;
    if((sphere->transparency>0||sphere->reflection>0)&&depth<MAX_RAY_DEPTH)
    {
		float facingratio = -raydir.dot(nhit);
        //�ı���ֵ��΢��Ч��
		float fresneleffect = mix(pow(1 - facingratio, 3), 1, 0.1);
        //���㷴�䷽��
		Vec3f refldir = raydir - nhit * 2 * raydir.dot(nhit);
		Vec3f reflection = trace(phit + nhit * bias, refldir, spheres, depth + 1);
		Vec3f refraction = 0;
        //�������Ҳ��͸���ģ����������ߣ�͸�䣩
        if(sphere->transparency)
        {
			float ior = 1.1, eta = (inside) ? ior : 1 / ior;//���ڱ������滹�����棿
			float cosi = -nhit.dot(raydir);
			float k = 1 - eta * eta * (1 - cosi * cosi);
			Vec3f refrdir = raydir * eta + nhit * (eta*cosi - sqrt(k));
			refrdir.normalize();
			refraction = trace(phit - nhit * bias, refrdir, spheres, depth + 1);
        }
        //����Ƿ��������Ļ��ֵ(���������͸����)
		surfaceColor = (reflection*fresneleffect +
			refraction * (1 - fresneleffect)*sphere->transparency)*sphere->surfaceColor;
    }
    else
    {
        //����һ�����������壬����Ҫ�ٽ��й���׷��
        for(unsigned i=0;i<spheres.size();++i)
        {
            if(spheres[i].emissionColor.x>0)
            {
                //��������
				Vec3f transmission = 1;
				Vec3f lightDirection = spheres[i].center - phit;
                lightDirection.normalize();
                for(unsigned j=0;j<spheres.size();++j)
                {
                    if(i!=j)
                    {
                        float t0,t1;
                        if(spheres[j].intersect(phit+nhit*bias,lightDirection,t0,t1))
                        {
                            transmission=0;
                            break;
                        }
                    }
                }
                surfaceColor+=sphere->surfaceColor*transmission*
                std::fmax(float(0),nhit.dot(lightDirection))*spheres[i].emissionColor;
            }
        }
    }
    return surfaceColor+sphere->emissionColor;
}


//����Ⱦ���������Ǽ���һ����������߶�ͼ���е�ÿ�����أ�׷�ٲ�������ɫ��
//������߼��������壬�����������ཻ�����ɫ�����򷵻ر���ɫ
void render(const std::vector<Sphere> &spheres)
{
	unsigned width = 1920, height = 1080;
	Vec3f* image = new Vec3f[width*height], *pixel = image;
	float invWidth = 1 / float(width), invHeight = 1 / float(height);
	float fov = 45, aspectratio = width / float(height);
	float angle = tan(M_PI*0.5*fov / 180);
    //׷�ٹ���
    for(unsigned y=0;y<height;++y)
    {
        for(unsigned x=0;x<width;++x,++pixel)
        {
			float xx = (2 * ((x + 0.5) * invWidth) - 1) * angle * aspectratio;
			float yy = (1 - 2 * ((y + 0.5) * invHeight)) * angle;
			Vec3f raydir(xx, yy, -1);
			raydir.normalize();
			*pixel = trace(Vec3f(0), raydir, spheres, 0);
        }
    }
    //������ΪPPM��ʽͼ��
    std::ofstream ofs("./MyRayTracerDemo(1st).ppm",std::ios::out|std::ios::binary);
    ofs<<"P6\n"<<width<<" "<<height<<"\n255\n";
    for (unsigned i = 0; i < width * height; ++i) { 
        ofs << (unsigned char)(std::fmin(float(1), image[i].x) * 255) << 
               (unsigned char)(std::fmin(float(1), image[i].y) * 255) << 
               (unsigned char)(std::fmin(float(1), image[i].z) * 255); 
    } 
    ofs.close(); 
    delete [] image; 
}


int main()
{
    srand(13);
    std::vector<Sphere> spheres;
    //λ�ã��뾶��������ɫ�������ʣ�͸���ȣ�������ɫ
    spheres.push_back(Sphere(Vec3f( 0.0, -10004, -20), 10000, Vec3f(0.20, 0.20, 0.20), 0, 0.0)); 
    spheres.push_back(Sphere(Vec3f( 0.0,      0, -20),     4, Vec3f(1.00, 0.32, 0.36), 1, 0.5)); 
    spheres.push_back(Sphere(Vec3f( 5.0,     -1, -15),     2, Vec3f(0.90, 0.76, 0.46), 1, 0.3)); 
    spheres.push_back(Sphere(Vec3f( 5.0,      5, -25),     3, Vec3f(0.65, 0.77, 0.07), 1, 0.4)); 
    spheres.push_back(Sphere(Vec3f(-5.5,      0, -15),     3, Vec3f(0.90, 0.90, 0.90), 1, 0.4));
	spheres.push_back(Sphere(Vec3f(12.0, -2, -20), 5, Vec3f(0.70, 0.50, 0.46), 1, 0.5));
    //��
    spheres.push_back(Sphere(Vec3f( 0.0,     20, -30),     3, Vec3f(0.00, 0.00, 0.00), 0, 0.0, Vec3f(3)));
    render(spheres);
    return 0;
}