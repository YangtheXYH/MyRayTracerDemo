#include <iostream>
#include <vector>
#include <fstream>
#include <cassert>
#include <cstdlib>
#include <cstdio>
#include <cmath>

#define M_PI 3.141592653589793
#define INFINITY 1e8

//自定义向量类
template<typename T>
class Vec3
{
public:
    T x,y,z;
    //初始化
    Vec3():x(T(0)),y(T(0)),z(T(0)){}
    Vec3(T xx):x(xx),y(xx),z(xx){}
    Vec3(T xx,T yy,T zz):x(xx),y(yy),z(zz){}
    Vec3& normalize() //归一化
    {
		T nor2 = length2();//见下定义
        if(nor2>0)
        {
			T invNor = 1 / sqrt(nor2);
			x *= invNor, y *= invNor, z *= invNor;
        }
        return *this;
    }
	Vec3<T> operator * (const T &f) const { return Vec3<T>(x*f, y*f, z*f); } //乘一个常量
	Vec3<T> operator * (const Vec3<T> &v) const { return Vec3<T>(x*v.x, y*v.y, z*v.z); } //乘一个向量
	Vec3<T> operator - (const Vec3<T> &v) const { return Vec3<T>(x - v.x, y - v.y, z - v.z); } //减一个向量
	Vec3<T> operator + (const Vec3<T> &v) const { return Vec3<T>(x + v.x, y + v.y, z + v.z); } //加一个向量
	Vec3<T>& operator += (const Vec3<T> &v) { x += v.x, y += v.y, z += v.z; return *this; }
	Vec3<T>& operator *= (const Vec3<T> &v)  { x *= v.x, y *= v.y, z *= v.z; return *this; }
	Vec3<T> operator - () const { return Vec3<T>(-x, -y, -z); } //取反
    T dot(const Vec3<T> &v) const {return x * v.x + y * v.y + z * v.z;} //向量点乘,点乘的几何含义我就不用说了
	T length2() const { return x * x + y * y + z * z; } //模^2
	T length() const { return sqrt(length2()); } //模

    friend std::ostream & operator << (std::ostream &os,const Vec3<T> &v)
    {
        os << "[" <<v.x<<" "<<v.y<<" "<<v.z<<"]";
        return os;
    }
};

typedef Vec3<float> Vec3f;

//自定义球体类

class Sphere
{
public:
    Vec3f center; //球体的位置
    float radius,radius2; //球体的半径和半径的平方
    Vec3f surfaceColor,emissionColor; //表面颜色和发散颜色
    float transparency,reflection; //表面透明度和反射率
    Sphere(
        const Vec3f &c,const float &r,
        const Vec3f &sc,const float &refl=0,
        const float &transp=0,const Vec3f &ec=0):
        center(c),radius(r),radius2(r*r),surfaceColor(sc),emissionColor(ec),
        transparency(transp),reflection(refl)
        {}

    //光与球面相交。这一段要好好理解
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


#define MAX_RAY_DEPTH 5 //最大递归深度

float mix(const float &a,const float &b,const float &mix)
{
	return b * mix + a * (1 - mix);
}

/*这一段是主要的追踪函数。取一条光线作为参数（由他的原点和方向定义）。测试这条光线
是否与场景中的任何几何图形相交。如果光线与一个物体相交，则计算相交点、相交点的
法向量，并用这些信息着色这个点。着色取决于物体表面的属性（是否透明、反光、漫反射）。
如果光线与交叉点处对象颜色的对象相交，则该函数返回光线的颜色，否则返回背景颜色。*/
Vec3f trace(const Vec3f &rayorig,const Vec3f &raydir,
            const std::vector<Sphere> &spheres,const int &depth)
{
    float tnear=INFINITY;
    const Sphere* sphere=NULL;
    //找到光线与场景中的球体的相交点
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
    //如果没有相交，则返回黑色或者背景色
    if(!sphere) return Vec3f(2);
	Vec3f surfaceColor = 0; //光线、与光线相交的物体表面的颜色
	Vec3f phit = rayorig + raydir * tnear; //相交点
	Vec3f nhit = phit - sphere->center;//相交点的法向量
    nhit.normalize(); //归一化法向量
    //如果法向量和观察方向不是相互相反的，反转法向量的方向。
    //这也意味着我们在球体内部，所以设置布尔量“是否在内部”
    //为“真”。最终，反转IdotN的标记
	float bias = 1e-1; //加入一些偏斜给我们将追踪的点
	bool inside = false;
	if (raydir.dot(nhit) > 0)
		nhit = -nhit, inside = true;
    if((sphere->transparency>0||sphere->reflection>0)&&depth<MAX_RAY_DEPTH)
    {
		float facingratio = -raydir.dot(nhit);
        //改变混合值以微调效果
		float fresneleffect = mix(pow(1 - facingratio, 3), 1, 0.1);
        //计算反射方向
		Vec3f refldir = raydir - nhit * 2 * raydir.dot(nhit);
		Vec3f reflection = trace(phit + nhit * bias, refldir, spheres, depth + 1);
		Vec3f refraction = 0;
        //如果球形也是透明的，计算折射线（透射）
        if(sphere->transparency)
        {
			float ior = 1.1, eta = (inside) ? ior : 1 / ior;//是在表面里面还是外面？
			float cosi = -nhit.dot(raydir);
			float k = 1 - eta * eta * (1 - cosi * cosi);
			Vec3f refrdir = raydir * eta + nhit * (eta*cosi - sqrt(k));
			refrdir.normalize();
			refraction = trace(phit - nhit * bias, refrdir, spheres, depth + 1);
        }
        //结果是反射和折射的混合值(如果球体是透明的)
		surfaceColor = (reflection*fresneleffect +
			refraction * (1 - fresneleffect)*sphere->transparency)*sphere->surfaceColor;
    }
    else
    {
        //它是一个漫反射物体，不需要再进行光线追踪
        for(unsigned i=0;i<spheres.size();++i)
        {
            if(spheres[i].emissionColor.x>0)
            {
                //这是束光
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


//主渲染函数。我们计算一束摄像机射线对图像中的每个像素，追踪并返回颜色。
//如果射线集中了球体，返回球体在相交点的颜色，否则返回背景色
void render(const std::vector<Sphere> &spheres)
{
	unsigned width = 1920, height = 1080;
	Vec3f* image = new Vec3f[width*height], *pixel = image;
	float invWidth = 1 / float(width), invHeight = 1 / float(height);
	float fov = 45, aspectratio = width / float(height);
	float angle = tan(M_PI*0.5*fov / 180);
    //追踪光线
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
    //保存结果为PPM格式图像
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
    //位置，半径，表面颜色，反射率，透明度，发射颜色
    spheres.push_back(Sphere(Vec3f( 0.0, -10004, -20), 10000, Vec3f(0.20, 0.20, 0.20), 0, 0.0)); 
    spheres.push_back(Sphere(Vec3f( 0.0,      0, -20),     4, Vec3f(1.00, 0.32, 0.36), 1, 0.5)); 
    spheres.push_back(Sphere(Vec3f( 5.0,     -1, -15),     2, Vec3f(0.90, 0.76, 0.46), 1, 0.3)); 
    spheres.push_back(Sphere(Vec3f( 5.0,      5, -25),     3, Vec3f(0.65, 0.77, 0.07), 1, 0.4)); 
    spheres.push_back(Sphere(Vec3f(-5.5,      0, -15),     3, Vec3f(0.90, 0.90, 0.90), 1, 0.4));
	spheres.push_back(Sphere(Vec3f(12.0, -2, -20), 5, Vec3f(0.70, 0.50, 0.46), 1, 0.5));
    //光
    spheres.push_back(Sphere(Vec3f( 0.0,     20, -30),     3, Vec3f(0.00, 0.00, 0.00), 0, 0.0, Vec3f(3)));
    render(spheres);
    return 0;
}