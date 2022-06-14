/*
	DBSCAN Algorithm
	15S103182
	Ethan
*/
#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <ctime>
#include <cstdlib>
#include <limits>
#include <cmath>
#include <stack>
#include <ros/ros.h>
using namespace std;
int clusterCount = 0;
class point{
public:
    double z;
    int cluster=0;
    int pointType=1;//1 noise 2 border 3 core
    int pts=0;//points in MinPts
    vector<int> corepts;
    int visited = 0;
    point()
    {

    }
    point (float a, int c)
    {
        z = a;
        cluster = c;
    }
};

void DBSCAN(vector<point> dataset,float Eps,int MinPts)
{
    int len = dataset.size();
    //calculate pts
    cout<<"calculate pts"<<endl;
    for(int i = 0; i < len; i ++)
    {
        for(int j = i + 1; j < len; j ++)
        {
            if(abs(dataset[i].z - dataset[j].z) < Eps)
            {
                dataset[i].pts++;
                dataset[j].pts++;
            }

        }
    }
    //core point
    cout<<"core point "<<endl;
    vector<point> corePoint;
    for(int i = 0; i < len; i ++)
    {
        if(dataset[i].pts >= MinPts)
        {
            dataset[i].pointType = 3;
            corePoint.push_back(dataset[i]);
        }
    }
    cout<<"joint core point"<<endl;
    //joint core point
    for(int i = 0; i<corePoint.size(); i++)
    {
        for(int j = i+1; j < corePoint.size(); j++)
        {
            if(abs(corePoint[i].z - corePoint[j].z) < Eps)
            {
                corePoint[i].corepts.push_back(j);
                corePoint[j].corepts.push_back(i);
            }
        }
    }
    for(int i=0;i < corePoint.size(); i++)
    {
        stack<point*> ps;
        if(corePoint[i].visited == 1)
        {
            continue;
        }
        ps.push(&corePoint[i]);
        point *v;
        while(!ps.empty())
        {
            v = ps.top();
            v->visited = 1;
            ps.pop();
            for(int j=0;j<v->corepts.size();j++)
            {
                if(corePoint[v->corepts[j]].visited==1)
                {
                    continue;
                }
                corePoint[v->corepts[j]].cluster = corePoint[i].cluster;
                clusterCount ++;
                corePoint[v->corepts[j]].visited = 1;
                ps.push(&corePoint[v->corepts[j]]);
            }
        }
    }
    cout<<"border point,joint border point to core point"<<endl;
    //border point,joint border point to core point
    for(int i=0; i<len; i++)
    {
        if(dataset[i].pointType==3) continue;
        for(int j=0;j<corePoint.size();j++)
        {
            if(abs(dataset[i].z - corePoint[j].z) < Eps)
            {
                dataset[i].pointType = 2;
                dataset[i].cluster = corePoint[j].cluster;
                break;
            }
        }
    }


    for(int i=0; i<len; i++)
    {
        if(dataset[i].pointType == 2)cout <<dataset[i].z <<"," <<dataset[i].cluster<<"\n";
    }
    for(int i=0;i<corePoint.size();i++)
    {
        cout << corePoint[i].z << "," << corePoint[i].cluster << "\n";
    }

}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "dbscan_node");
    //声明节点句柄
    ros::NodeHandle nh;
    vector<point> dataset;
    for(int i = 0; i < 10; i ++)
    {
        point onePoint;
        cin >> onePoint.z;
        dataset.push_back(onePoint);
    }
    DBSCAN(dataset,1.5,2);
    return 0;
}