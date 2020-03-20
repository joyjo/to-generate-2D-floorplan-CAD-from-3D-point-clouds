#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/octree/octree.h>

#include <iostream>
#include <fstream>

// opencv library
#include <opencv2/opencv_modules.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/ximgproc.hpp>
#include <opencv2/line_descriptor/descriptor.hpp>

// 
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPtr;
// 
const double stepSize = 0.01;
const double ceilingThr = 0.3;
// ransac
const int maxItr = 500; 
const double distThr = 0.1;
// out path
const std::string outPath = "out/";

// Pass through filter
PointCloudPtr filterPassThrough(PointCloudPtr cloud, std::string axis, float limit1, float limit2)
{
    PointCloudPtr filtered (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName (axis);
    pass.setFilterLimits (limit1, limit2);
    pass.filter (*filtered);
    // std::cerr << "(Pass through filter)Input has: " << filtered->width * filtered->height << " data points." << std::endl;
    
    return filtered;
}
// Statistical Outlier Removal Filter
PointCloudPtr filterOutlierRemoval(PointCloudPtr cloud)
{
    PointCloudPtr filtered (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(8);
    sor.setStddevMulThresh(0);
    sor.filter(*filtered);

    std::cerr << "(Statistical Outlier Removal Filter)Input has: " << filtered->width * filtered->height << " data points." << std::endl;
    return filtered;
}
// Vox downsample the dataset using a leaf size of 1cm
PointCloudPtr downsamplingVoxel(PointCloudPtr cloud)
{
    PointCloudPtr filtered (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::VoxelGrid<pcl::PointXYZ> vox;
    vox.setInputCloud (cloud);
    vox.setLeafSize (0.01f, 0.01f, 0.01f); // sampling 1cm
    vox.filter (*filtered);

    std::cerr << "(Downsampled)Input has: " << filtered->width * filtered->height << " data points." << std::endl;
    return filtered;
}

void writeFloorCeiling(PointCloudPtr cloud)
{
    pcl::PCDReader reader;
    pcl::PCDWriter writer;
    PointCloudPtr floorPtr (new pcl::PointCloud<pcl::PointXYZ>), 
                        ceilingPtr (new pcl::PointCloud<pcl::PointXYZ>), 
                        filtered (new pcl::PointCloud<pcl::PointXYZ>);

    reader.read(outPath + "floor.pcd", *floorPtr);
    reader.read(outPath + "ceiling.pcd", *ceilingPtr);

    pcl::PointXYZ flMin, flMax, clMin, clMax;
    pcl::getMinMax3D(*floorPtr, flMin, flMax);
    pcl::getMinMax3D(*ceilingPtr, clMin, clMax);

    filtered = filterPassThrough(cloud, "z", flMax.z, clMin.z);

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (flMin.z, flMax.z);
    pass.setFilterLimits (clMin.z, clMax.z);
    pass.filter (*filtered);

    writer.write (outPath + "floorceiling.pcd", *filtered, false);
}

PointCloudPtr removeFloorCeilingPlanes(PointCloudPtr cloud)
{
    PointCloudPtr floorPtr (new pcl::PointCloud<pcl::PointXYZ>);
    PointCloudPtr ceilingPtr (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    seg.setOptimizeCoefficients (true);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations(maxItr); // default 50
    seg.setDistanceThreshold(distThr); //10cm
    seg.setAxis(Eigen::Vector3f(0.0 * -1.0, 0.0, 1.0)); //per to z axis
    seg.setEpsAngle( 20.0f * (M_PI/180.0f) );

    int co = 0;
    pcl::PointXYZ maxPt;
    // get 2 biggest horizontal plane
    while(co!=2){
        // Fit a plane
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.size() == 0)
            break;

        // Extract inliers
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        PointCloudPtr extractPlanPtr (new pcl::PointCloud<pcl::PointXYZ>);
        extract.filter (*extractPlanPtr);
        
        pcl::PointXYZ tmpMinPt, tmpMaxPt;
        pcl::getMinMax3D(*extractPlanPtr, tmpMinPt, tmpMaxPt);
         
        if(co == 0){
            ceilingPtr = extractPlanPtr;
            maxPt = tmpMaxPt;
        } else {
            if(maxPt.z > tmpMaxPt.z){
                floorPtr = extractPlanPtr;
            } else {
                floorPtr = ceilingPtr;
                ceilingPtr = extractPlanPtr;
            }
        }

        extract.setNegative(true);
        pcl::PointCloud<pcl::PointXYZ> tmp;
        extract.filter(tmp);
        cloud->swap(tmp);

        co++;
    }

    pcl::PCDWriter writer;
    std::cerr << "PointCloud representing the ceiling planar component: " << ceilingPtr->width * ceilingPtr->height << " data points." << std::endl;
    writer.write (outPath + "ceiling.pcd", *ceilingPtr, false);

    std::cerr << "PointCloud representing the floor planar component: " << floorPtr->width * floorPtr->height << " data points." << std::endl;
    writer.write (outPath + "floor.pcd", *floorPtr, false);

    writeFloorCeiling(cloud);

    return cloud;
}

void extractWallPlanes(PointCloudPtr cloud)
{
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    seg.setOptimizeCoefficients (true);
    seg.setModelType(pcl::SACMODEL_PARALLEL_PLANE); //SACMODEL_PERPENDICULAR_PLANE //SACMODEL_PARALLEL_PLANE //SACMODEL_PLANE
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations(500); // default 50
    seg.setDistanceThreshold(0.5); //10cm
    // seg.setAxis (Eigen::Vector3f (0.0 * -1.0, 1.0, 0.0)); //par zx axis
    seg.setAxis(Eigen::Vector3f (0.0 * -1.0, 0.0, 1.0));
    seg.setAxis(Eigen::Vector3f (0.0, 0.0 * -1.0, 1.0));
    seg.setEpsAngle( 10.0f * (M_PI/180.0f) );

    // calculate floor height
    pcl::PCDReader reader;
    PointCloudPtr floorPtr (new pcl::PointCloud<pcl::PointXYZ>), 
                        ceilingPtr (new pcl::PointCloud<pcl::PointXYZ>);
    reader.read(outPath + "floor.pcd", *floorPtr);
    reader.read(outPath + "ceiling.pcd", *ceilingPtr);

    pcl::PointXYZ flMin, flMax, clMin, clMax;
    pcl::getMinMax3D(*floorPtr, flMin, flMax);
    pcl::getMinMax3D(*ceilingPtr, clMin, clMax);
    int heightThr = 0.8*(flMax.z - clMin.z);
    // end calculate floor height

    int original_size(cloud->height*cloud->width);
    int co = 0;
    pcl::PCDWriter writer;
    while (cloud->height*cloud->width>0.03*original_size){
        // Fit a plane
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.size() == 0)
            break;

        // Extract inliers
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        PointCloudPtr extractPlanPtr (new pcl::PointCloud<pcl::PointXYZ>);
        extract.filter (*extractPlanPtr);

        bool bo=false;
        pcl::PointXYZ tmpMinPt, tmpMaxPt;
        pcl::getMinMax3D(*extractPlanPtr, tmpMinPt, tmpMaxPt);

        // check height
        if( (tmpMaxPt.z-tmpMinPt.z) >  heightThr) bo = true;

        if(bo)
        {
            writer.write (outPath + "plane/plane_"+std::to_string(co)+".pcd", *extractPlanPtr, false);
            std::cerr << "PointCloud representing the (wall)planar component: " << extractPlanPtr->width * extractPlanPtr->height << " data points." << std::endl;
        }

        extract.setNegative(true);
        pcl::PointCloud<pcl::PointXYZ> tmp;
        extract.filter(tmp);
        cloud->swap(tmp);

        if(co==3) break;

        co++;
    }

    writer.write (outPath + "plane/wall1.pcd", *cloud, false);
}

cv::Mat PCL2Mat(PointCloudPtr cloud, std::string removeax)
{
    pcl::PointXYZ c2Min, c2Max;
    pcl::getMinMax3D(*cloud, c2Min, c2Max);

    std::string ax1, ax2;
    float ax1Max, ax1Min, ax2Max, ax2Min;

    if(removeax=="x")
    {
        ax1 = "y";
        ax2 = "z";
        ax1Min = c2Min.y;
        ax1Max = c2Max.y;
        ax2Min = c2Min.z;
        ax2Max = c2Max.z;
    }
    else if(removeax=="y")
    {
        ax1 = "x";
        ax2 = "z";
        ax1Min = c2Min.x;
        ax1Max = c2Max.x;
        ax2Min = c2Min.z;
        ax2Max = c2Max.z;
    }
    else if(removeax=="z")
    {
        ax1 = "x";
        ax2 = "y";
        ax1Min = c2Min.x;
        ax1Max = c2Max.x;
        ax2Min = c2Min.y;
        ax2Max = c2Max.y;
    }

    std::vector<std::vector<PointCloudPtr>> grid;
    int maxPoints = 0;
    for (float i = ax1Min; i < ax1Max; i += stepSize)
    {
        PointCloudPtr slice = filterPassThrough(cloud, ax1, i, i + stepSize);

        std::vector<PointCloudPtr> slicePoint;
        for (float j = ax2Min; j < ax2Max; j += stepSize)
        {
            PointCloudPtr gridCell = filterPassThrough(slice, ax2, j, j + stepSize);

            int gridSize = gridCell->points.size();
            slicePoint.push_back(gridCell);

            if (gridSize > maxPoints)
            {
                maxPoints = gridSize;
            }
        }
        grid.push_back(slicePoint);
    }

    cv::Mat img(static_cast<int>(grid.size()), static_cast<int>(grid.at(0).size()), CV_8UC1);
    img = cv::Scalar(0);
    for (int i = 0; i < img.rows; ++i)
    {
        for (int j = 0; j < img.cols; ++j)
        {
            int pointCount = grid.at(i).at(j)->points.size();
            float percentOfMax = (pointCount + 0.0) / (maxPoints + 0.0);
            int intensity = percentOfMax * 255;
            img.at<uchar>(i, j) = intensity;
        }
    }
    
    return img;
}

PointCloudPtr Mat2PCL(cv::Mat img, std::vector<std::vector<PointCloudPtr>> gridCloud)
{
    PointCloudPtr wallPoint(new pcl::PointCloud<pcl::PointXYZ>);

    for(int i=0; i<img.rows; ++i)
    {
        for(int j=0; j<img.cols; ++j)
        {
            cv::Vec3b c = img.at<cv::Vec3b>(cv::Point (j, i));
            if(c[2]>70 && c[2]<255)
            {
                PointCloudPtr tmpgridCell = gridCloud.at(i).at(j);
                int nGridCell = tmpgridCell->points.size();
                for (int k = 0; k < nGridCell; k++) 
                {
                    wallPoint->push_back(tmpgridCell->points[k]);
                }
            }
            
        }
    }

    pcl::PCDWriter writer;
    writer.write (outPath + "re_wall_points.pcd", *wallPoint, false);
    std::cerr << "Re wall has: " << wallPoint->width * wallPoint->height << " data points." << std::endl;

    return wallPoint;
}

PointCloudPtr createWallPCD(PointCloudPtr cloud)
{
    PointCloudPtr tmp (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCDWriter writer;

    // voxelization(filtering)
    // tmp = downsamplingVoxel(cloud);
    //  tmp = filterPassThrough(cloud)
    // writer.write (outPath + "voxed_points.pcd", *tmp, false);
    tmp = removeFloorCeilingPlanes(cloud);
    writer.write (outPath + "wall_points.pcd", *tmp, false);

    return tmp;
}

PointCloudPtr removeOutliersFromWallPCD(PointCloudPtr cloud, int type=0)
{
    pcl::PCDReader reader;
    pcl::PCDWriter writer;
    PointCloudPtr floorPtr (new pcl::PointCloud<pcl::PointXYZ>), 
                        ceilingPtr (new pcl::PointCloud<pcl::PointXYZ>), 
                        filtered (new pcl::PointCloud<pcl::PointXYZ>);

    reader.read(outPath + "floor.pcd", *floorPtr);
    reader.read(outPath + "ceiling.pcd", *ceilingPtr);

    pcl::PointXYZ flMin, flMax, clMin, clMax;
    pcl::getMinMax3D(*floorPtr, flMin, flMax);
    pcl::getMinMax3D(*ceilingPtr, clMin, clMax);

    if(type==0) // filter not between ceiling and floor
    {
        filtered = filterPassThrough(cloud, "z", flMax.z, clMin.z);
        //filtered = filterPassThrough(filtered, "y", flMin.y, flMax.y);
        //filtered = filterPassThrough(filtered, "x", flMin.x, flMax.x);
    }

    if(type==1) // filter ceiling at 30cm
    {
        double tmpAt30Ceiling = clMin.z - ceilingThr;
        filtered = filterPassThrough(cloud, "z", tmpAt30Ceiling, clMin.z); // filter at 30cm from ceiling
    } 

    writer.write (outPath + "filtered.pcd", *filtered, false);

    return filtered;
}

cv::Mat createDepthImg(PointCloudPtr cloud)
{
    cv::Mat img, gray;
    img = PCL2Mat(cloud, "z");
    cv::imwrite(outPath + "img/depth.jpg", img);

    Canny(img, gray, 50, 200, 3);
    cv::imwrite(outPath + "img/gray.jpg", gray);

    return img;
}

cv::Mat houghLineDetection(cv::Mat img)
{
    cv::Mat dst, cdst;

    Canny(img, dst, 50, 200, 3);
    cv::imwrite(outPath + "img/gray.jpg", dst);

    cvtColor(dst, cdst, cv::COLOR_GRAY2BGR); //CV_GRAY2BGR v2

    std::vector<cv::Vec4i> lines;
    HoughLinesP(dst, lines, 1, CV_PI/180, 50, 50, 10 );
    for( size_t i = 0; i < lines.size(); i++ )
    {
        cv::Vec4i l = lines[i];
        line( cdst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 3, cv::LINE_AA); //CV_AA v2
    }
    cv::imwrite(outPath + "img/hld.jpg", cdst);

    return cdst;
}

cv::Mat fastLineDetection()
{
    cv::Mat img = cv::imread(outPath + "img/depth.jpg", cv::IMREAD_GRAYSCALE);

    std::vector<cv::Vec4f> lines;
    cv::Ptr<cv::ximgproc::FastLineDetector> d = cv::ximgproc::createFastLineDetector(10, 1.41421356f, 50.0, 50.0, 3, true);
    d->detect(img, lines);

    cv::Mat cdst(img);
    d->drawSegments(cdst, lines);

    cv::imwrite(outPath + "img/fld.jpg", cdst);

    return cdst;
}

// start line detection
bool isSameLine(int s0, int e0, int s1, int e1)
{
    bool bo=false;
    int co=0;
    for(int i=s1; i<e1+1; i++)
    {
        for(int j=s0; j<e0+1; j++)
        {
            if(i==j) co++;
        }
    }

    int ln1 = e0-s0, ln2 = e1-s1;
    if( (ln1<ln2 && 100*co/ln1 > 80) || (ln1>=ln2 && 100*co/ln2 > 80) ) bo=true;

    return bo;
}

bool isW0(cv::Mat img, int col, int row, int dir=0)
{
    bool bo=false;
    cv::Point pt=cv::Point(col, row);
    if(dir==1) pt=cv::Point(row, col);

    cv::Vec3b c = img.at<cv::Vec3b>(pt);
    if(c[0] > 0 && c[1] > 0 && c[2] > 0)
        bo=true;

    return bo;
}

cv::Mat drawLine(cv::Mat img, cv::Point pt1, cv::Point pt2, int dir=0, int type=0)
{
    int linethin=1;
    if(type==1) linethin=3;

    line(img ,pt1, pt2, cv::Scalar(255,255,255), linethin, cv::LINE_AA);

    // points -> file 
    if(type==1)
    {
        std::string d = "x";
        double l = (pt2.x-pt1.x)/100.0; // in image cm
        double x0 = pt1.x/100.0;
        double y0 = pt1.y/100.0;

        if(dir==0)
        {
            d = "y";
            l = (pt2.y-pt1.y)/100.0;
        }

        std::ofstream f;
        f.open (outPath + "coord/walls.txt", std::ios_base::app);
        f << x0 << " " << y0 << " " <<  l << " " << d <<  "\n";
        f.close();
    }

    return img;
}

/**
 * dir, 0 vertical, 1 horizontal
 * step2, range of line
 **/
std::vector<int> detectLines(cv::Mat depth, int dir=0, int step2=50)
{
    std::vector<int> coords;
    int step1=1, size1=depth.cols, size2=depth.rows;
    if(dir==1)
    {
        size1=depth.rows;
        size2=depth.cols;
    }

    for(int i=0; i<size1; i+=step1)
    {
        bool bo = false;
        for(int j=0;j<size2; j++)
        {
            if(isW0(depth, i, j, dir)){
                bo = true;
                break;
            }
        }

        if(bo) //if at least one point
        {
            int co=0;
            for(int k=i; k<i+step2; k++)
            {
                for(int m=0; m<size2; m++)
                {
                    if(isW0(depth, k, m, dir))
                        co++;
                }
            }
            
            if(co>50*step2)
            {
                coords.push_back(i); // save start point in line
                step1 = step2;
            }
        }
    }

    return coords;
}

std::vector<std::pair<cv::Point, cv::Point>> mergeWallLines(std::vector<std::pair<cv::Point, cv::Point>> lines, int dir)
{
    // merge parallel lines
    std::vector<std::pair<cv::Point, cv::Point>> newlines;
    std::vector<int> samelines;
    for(int i=0; i<lines.size(); i++)
    {
        if( std::find(samelines.begin(), samelines.end(), i) != samelines.end() ) continue;

        int tmpx00=lines.at(i).first.x,
                tmpy00=lines.at(i).first.y, 
                    tmpy01=lines.at(i).second.y;
        if(dir==1)
        {
            tmpx00=lines.at(i).first.y;
            tmpy00=lines.at(i).first.x;
            tmpy01=lines.at(i).second.x;
        }

        bool bo = true;
        for(int j=0; j<lines.size(); j++)
        {
            if(i==j) continue;

            int tmpx10=lines.at(j).first.x,
                    tmpy10=lines.at(j).first.y, 
                        tmpy11=lines.at(j).second.y;
            if(dir==1)
            {
                tmpx10=lines.at(j).first.y;
                tmpy10=lines.at(j).first.x;
                tmpy11=lines.at(j).second.x;
            }

            int dist = tmpx00 - tmpx10;
            if( std::abs(dist)<=50 && isSameLine(tmpy00, tmpy01, tmpy10, tmpy11) )
            {
                // merge to big one
                int ln1=tmpy11-tmpy10, ln2=tmpy01-tmpy00;
                if( ln1-ln2 > 0 ) bo = false;
                if( ln1==ln2 ) samelines.push_back(j);
                break;
            }
        }
        if(bo) newlines.push_back(lines.at(i));
    }
    // end merge parallel lines

    return newlines;
}
/** 
 * thre
 * min line len 200
 * point range step2 50
**/
cv::Mat detectWallLines(cv::Mat depth, cv::Mat newImg, std::vector<int> p, int dir=0, int step2=50)
{
    int size1=depth.cols, size2=depth.rows;
    if(dir==1)
    {
        size1=depth.rows;
        size2=depth.cols;
    }

    std::vector<std::pair<cv::Point, cv::Point>> lines;
    for(int i=0; i<p.size(); i++)
    {
        int arr[size2];
        int idx=p.at(i);  
        for(int j=0; j<size2; j++)
        {
            int co=0;
            // step2 x step2 area
            for(int m=j; m<(j+step2) && m<size2; m++)
            {
                // draw line middle of points
                for(int k=idx; k<(idx+step2-1) && k<size1; k++)
                {
                    if(isW0(depth, k, m, dir))
                        co++;
                }
            }
            arr[j]=co;
        }
        
        for(int ii=0; ii<size2; ii++)
        {
            if(arr[ii]>0)
            {
                int jj=ii, cco=0;
                for(; jj<size2; jj++)
                {
                    if(arr[jj]==0) break;
                    cco++;
                }
                
                if(cco>200)// line len > 200
                {
                    cv::Point pt1=cv::Point(idx, ii), pt2=cv::Point(idx, jj);
                    if(dir==1)
                    {
                        pt1=cv::Point(ii, idx), pt2=cv::Point(jj, idx);
                    }
                    lines.push_back( std::make_pair(pt1, pt2) );
                }
                ii=jj;
            }
        }
    }

    std::vector<std::pair<cv::Point, cv::Point>> newlines = mergeWallLines(lines, dir);

    //draw lines
    for(int m=0; m<newlines.size(); m++)
    {
       newImg=drawLine(newImg, newlines.at(m).first, newlines.at(m).second, dir, 1);
    }

    return newImg;
}

void lineDetection(cv::Mat img)
{
    std::vector<int> vert, horiz;
    cv::Mat wallImg;

    vert = detectLines(img, 0); //vertical candidate
    horiz = detectLines(img, 1);//horizontal candidate
    
    cv::Mat tmpImg2(img.rows, img.cols, CV_8UC1);
    tmpImg2 = detectWallLines(img, tmpImg2, vert, 0); //vertical wall
    wallImg = detectWallLines(img, tmpImg2, horiz, 1); //horizontal
    cv::imwrite(outPath + "img/walls.jpg", wallImg);
}

int main (int argc, char** argv)
{   
    PointCloudPtr inputCloud (new pcl::PointCloud<pcl::PointXYZ>), cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCDReader reader;
    cv::Mat depthImg;
    
    reader.read ("in/cloud.pcd", *inputCloud);
    std::cerr << "Input has: " << inputCloud->width * inputCloud->height << " data points." << std::endl;
    cloud = createWallPCD(inputCloud);
    cloud = removeOutliersFromWallPCD(cloud, 1);
    depthImg = createDepthImg(cloud);

    depthImg = cv::imread(outPath + "img/depth.jpg");
    lineDetection(depthImg);

    return 0;
}