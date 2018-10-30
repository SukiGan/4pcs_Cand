/*
 * Description::Move the Point one to Point two2
 * author::Cand
 * Data::2018--10--30
 * Modify::
 * add::
*/
#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/PCLPointCloud2.h>
#include <math.h>
#include <vector>

using namespace std;

typedef struct PointXYZa{
    double x;
    double y;
    double z;

}PointXYZa;

#define theld 1e-2
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

void printPointXYZ(const PointCloud::Ptr cloud);

int readTxtInfo(string* fileName, vector<PointXYZa> *a);

bool pcsCalDistance(const PointCloud::Ptr cloudSrc,const char* filename, vector<double> *r,
                    vector<PointXYZa> *a,vector<PointXYZa> *b,vector<PointXYZa> *c);

double pcsCalDistanceRate(const PointXYZa a,const PointXYZa b,const PointXYZa c,const PointXYZa d);

PointXYZa pcsCalPointOfLine(const PointXYZa a,const PointXYZa b,const  PointXYZa c);

void getPanel(PointXYZa p1,PointXYZa p2, PointXYZa p3,
              double &a, double &b, double &c, double &d);

void getNormal(PointXYZa p1,PointXYZa p2, PointXYZa p3,
               double &a, double &b, double &c, double &d);

double disPoint2Panel(PointXYZa p1,double a, double b, double c,double d);

//cal the mean of Matrix
void CalMeanOfPoint(vector<PointXYZa> &p,PointXYZa &mean);
//cal the pan Matrix
void CalPanMatrix(vector<PointXYZa> &p, PointXYZa &transform);

//To use transform Matrxi 
void transformPoint(PointXYZa &T,const PointCloud::Ptr cloudSrc);

PointXYZa pcsCalFindSamePoint();
int  main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOne(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTwo(new pcl::PointCloud<pcl::PointXYZ>);
    long *p;
    vector<double> disR1; //the R of pointdata one
    vector<double> disR2; //the R of pointdata two


    vector<PointXYZa> pointa1;
    vector<PointXYZa> pointb1;
    vector<PointXYZa> pointc1;

    vector<PointXYZa> pointa2;
    vector<PointXYZa> pointb2;
    vector<PointXYZa> pointc2;

    //Judge Input point data
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("NodePoint1.pcd", *cloudOne) == -1)
    {
        PCL_ERROR("Cloud not read file a_1.pcd \n");
        return (-1);
    }
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("NodePoint2.pcd", *cloudTwo) == -1)
    {
        PCL_ERROR("Cloud not read file a_2.pcd \n");
        return (-1);
    }
    //cal the fisrt point data distance
    pcsCalDistance(cloudOne,"test1.txt",&disR1,&pointa1,&pointb1,&pointc1);
    //cal the second point data distance
    pcsCalDistance(cloudTwo,"test2.txt",&disR2,&pointa2,&pointb2,&pointc2);

    vector<double>::iterator it1=disR1.begin();
    vector<double>::iterator it2;
    long index = 0;
    long count = 0;
    long same = 0;

    vector<PointXYZa> mean;

    PointXYZa meanResult1;
    PointXYZa meanResult2;

    while(it1 < disR1.end())
    {
        it2=disR2.begin();
        count=0;
        vector<PointXYZa> meanPoint1;
        vector<PointXYZa> meanPoint2;
        while(it2 < disR2.end())
        {
            //cout << *it1 << "   "<< *it2 << endl;

            //have question
            if (fabs(*it2-*it1) < theld)//
            {
                   //cout << endl << endl;
                   cout << *it1 << "   "<< *it2 << endl;
                   //print the first point
                   //cout << "the" << count << endl;
                   cout << pointa1[index].x << "  " << pointa1[index].y << "  "<< pointa1[index].z << endl;
                   cout << pointb1[index].x << "  " << pointb1[index].y << "  "<< pointb1[index].z << endl;
                   cout << pointc1[index].x << "  " << pointc1[index].y << "  "<< pointc1[index].z << endl;
                   cout << endl;
                   //print the second point
                   cout << pointa2[count].x << "  " << pointa2[count].y << "  "<< pointa2[count].z << endl;
                   cout << pointb2[count].x << "  " << pointb2[count].y << "  "<< pointb2[count].z << endl;
                   cout << pointc2[count].x << "  " << pointc2[count].y << "  "<< pointc2[count].z << endl;
                   meanPoint1.push_back(pointa1[index]);
                   meanPoint1.push_back(pointb1[index]);
                   meanPoint1.push_back(pointc1[index]);

                   meanPoint2.push_back(pointa2[count]);
                   meanPoint2.push_back(pointb2[count]);
                   meanPoint2.push_back(pointc2[count]);

                   CalMeanOfPoint(meanPoint1,meanResult1);
                   CalMeanOfPoint(meanPoint2,meanResult2);

                   cout << endl << endl << endl;
                   cout << meanResult1.x << "  "<< meanResult1.y << "  "<< meanResult1.z << endl;
                   cout << meanResult2.x << "  "<< meanResult2.y << "  "<< meanResult2.z << endl;
                   cout << endl << endl << endl;
                   mean.push_back(meanResult1);
                   mean.push_back(meanResult2);
                   same++;

            }           
            meanPoint1.clear();
            meanPoint2.clear();
            count++;
            it2++;
        }
        index++;
        it1++;
    }
    
    PointXYZa transf;
    //Get the Pan Matrix
    cout << "Transform Matrix::" << endl;
    cout << mean[0].x << "  " << mean[0].y << "  " << mean[0].z << endl;
    cout << mean[1].x << "  " << mean[1].y << "  " << mean[1].z << endl;
    CalPanMatrix(mean,transf);

    cout << transf.x << "  " << transf.y << "  " << transf.z << endl;

    //clear all memory
    pointa1.clear();
    pointa2.clear();
    pointb1.clear();
    pointb2.clear();
    pointc1.clear();
    pointc2.clear();
    disR1.clear();
    disR2.clear();
    mean.clear();
    cout << same << endl;
    //cout << "have" << count << "same"<< endl;
    //pcl::visualization::CloudViewer viewer("Simple viewer ");
    //viewer.showCloud(cloudOne);
    //printPointXYZ(cloudOne);
    //while( !viewer.wasStopped())
    //{
    //}

    return 0;
}

//the function print the data x y z information
//input the point cloud
//output---
void printPointXYZ(const PointCloud::Ptr cloud)
{
    for (int i =0; i < cloud->width*cloud->height; i++)
    {
        cout << cloud->points[i].x << "  ";
        cout << cloud->points[i].y << "  ";
        cout << cloud->points[i].z << endl;
    }

}
//the function cal the distance between one point and point
//input point data  the save path of file
bool pcsCalDistance(const PointCloud::Ptr cloudSrc,const char* filename,vector<double> *r,
                    vector<PointXYZa> *a,vector<PointXYZa> *b,vector<PointXYZa> *c)
{
    PointCloud::Ptr src(new PointCloud);
    src = cloudSrc;
    ofstream ofile;
    ofile.open(filename);
    //PointCloud::Ptr tgt(new PointCloud);
    //the four Point
    PointXYZa pointOne;
    PointXYZa pointTwo;
    PointXYZa pointThr;
    PointXYZa pointFou;
    pointFou.x=1;
    pointFou.y=1;
    pointFou.z=1;

    // the distance
    double dis;
    //the Ax+By+Cz+D=0 panel
    double A=0.0;
    double B=0.0;
    double C=0.0;
    double D=0.0;
    //count the the calculate
    long index=0;


    //tgt = cloudTgt;

    for (int i =0; i < (src->width*src->height); i++)
    {
        for (int j=i+1; j < (src->width*src->height); j++)
        {
            for (int k=j+1; k < (src->width*src->height); k++)
            {
                pointOne.x=src->points[i].x;pointOne.y=src->points[i].y;pointOne.z=src->points[i].z;
                pointTwo.x=src->points[j].x;pointTwo.y=src->points[j].y;pointTwo.z=src->points[j].z;
                pointThr.x=src->points[k].x;pointThr.y=src->points[k].y;pointThr.z=src->points[k].z;

                //getPanel(pointOne,pointTwo,pointThr,A,B,C,D);

                //for (int m=k+1; m < (src->width*src->height); m++)
                {
                    //pointFou.x=src->points[m].x;pointFou.y=src->points[m].y;pointFou.z=src->points[m].z;
                    dis=pcsCalDistanceRate(pointOne, pointTwo, pointThr, pointFou);
                    //dis=disPoint2Panel(pointFou,A,B,C,D);                  
                    r->push_back(dis);
                    a->push_back(pointOne);
                    b->push_back(pointTwo);
                    c->push_back(pointThr);
                    //cout << dis << endl;
                    //ofile << dis <<endl;
                    index++;
                }
            }
       }
    }

    ofile.close();
    cout << index << endl;

    return true;
}

//the function to cal  the three point distance
double pcsCalDistanceRate(const PointXYZa a,const PointXYZa b,const PointXYZa c,const PointXYZa d)
{
    //calculate the three point distance
   double disAB=sqrt((a.x-b.x)*(a.x-b.x)+
                   (a.y-b.y)*(a.y-b.y)+
                   (a.z-b.z)*(a.z-b.z));
   double disAC=sqrt((a.x-c.x)*(a.x-c.x)+
                     (a.y-c.y)*(a.y-c.y)+
                     (a.z-c.z)*(a.z-c.z));

   double disR;
   disR = disAB/disAC;

   return disR;
}

//the function cal the
PointXYZa pcsCalPointOfLine(const PointXYZa a, const PointXYZa b, const PointXYZa c)
{

}

//Function: get the panel equation
void getPanel(PointXYZa p1, PointXYZa p2, PointXYZa p3, double &a, double &b, double &c, double &d)
{
    a=(p2.y - p1.y)*(p3.z - p1.z) - (p2.z - p1.z)*(p3.y - p1.y);
    b=(p2.z - p1.z)*(p3.x - p1.x) - (p2.x - p1.x)*(p3.z - p1.z);
    c=(p2.x - p1.x)*(p3.y - p1.y) - (p2.y - p1.y)*(p3.x - p1.x);

    d=0-(a*p1.x+b*p1.y+c*p1.z);
}

//Function: Gett the Normal
void  getNormal(PointXYZa p1, PointXYZa p2, PointXYZa p3,double &a, double &b, double &c, double &d)
{
    a=(p2.y - p1.y)*(p3.z - p1.z) - (p2.z - p1.z)*(p3.y - p1.y);
    b=(p2.z - p1.z)*(p3.x - p1.x) - (p2.x - p1.x)*(p3.z - p1.z);
    c=(p2.x - p1.x)*(p3.y - p1.y) - (p2.y - p1.y)*(p3.x - p1.x);

    //return Vec3(a,b,c);
}

//function: cal the distance between point and panel
double disPoint2Panel(PointXYZa p1, double a, double b, double c,double d)
{

    return fabs(a*p1.x+ b*p1.y +  c*p1.z +d)/ sqrt(a*a + b*b + c*c);
}
//function:: calculate the mean of three point
void CalMeanOfPoint(vector<PointXYZa> &p1, PointXYZa &mean)
{
    vector<PointXYZa>::iterator it;
    mean.x = 0;
    mean.y = 0;
    mean.z = 0;

    for (it=p1.begin(); it!=p1.end(); it++)
    {
        mean.x += it->x;
        mean.y += it->y;
        mean.z += it->z;
    }

    mean.x = mean.x/3;
    mean.y = mean.y/3;
    mean.z = mean.z/3;
}

//function::  calculate the pan matrix of three Point
void CalPanMatrix(vector<PointXYZa> &p, PointXYZa &transform)
{
    cout << p[0].x << "  " << p[0].y << "  " << p[0].z << endl;
    cout << p[1].x << "  " << p[1].y << "  " << p[1].z << endl;
    transform.x = p[1].x-p[0].x;
    transform.y = p[1].y-p[0].y;
    transform.z = p[1].z-p[0].z;
}

int readTxtInfo(string* fileName,vector<PointXYZa>* a)
{
	
}


