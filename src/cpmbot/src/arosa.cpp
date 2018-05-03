#include <iostream>
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/common/common.h>
#include "sensor_msgs/Imu.h"
//#include "../include/mybot_msg/msgMybot_basicMovement.h"
#include <cpmbot/cpmbot_basic.h>
#include "nav_msgs/Odometry.h"

#include <pcl/common/centroid.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_datatypes.h>
#include <pcl_conversions/pcl_conversions.h>


#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <gazebo_msgs/LinkStates.h>
//convenient typedefs
/*
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;
*/
/*
// Define a new point representation for < x, y, z, curvature >
class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>
{
  using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
public:
  MyPointRepresentation ()
  {
    // Define the number of dimensions
    nr_dimensions_ = 4;
  }

  // Override the copyToFloatArray method to define our feature vector
  virtual void copyToFloatArray (const PointNormalT &p, float * out) const
  {
    // < x, y, z, curvature >
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
    out[3] = p.curvature;
  }
};*/

ros::Publisher pub,pub2,pub3,pub4;   // Variables para los elementos publicadores
double ultimo_angulo=0;              // Almacenaremos el ultimo angulo del brazo trasero
double ultimo_angulo_delantero=0;    // Almacenaremos el ultimo angulo del brazo delanero 
double actualTime;                   // Para algunos procesos, almacenaremos la hora actual
double lastLoopTime;                 // Para poder compararla con la de lastlooptime
int verdes=0;                        // Variable donde almacenaremos la cantidad de punos a una altura pasable que hemos encontrado 
cpmbot::cpmbot_basic cmdBasicMovement_; // Variable necesaria para poder enviar mensajes con comandos de movimiento
enum Estado { inicio, suelo, subiendo, arriba , bajando };  // Enumeración de los estados posibles
Estado estado=suelo;  // Variable para almacenar el estado actual del sistema  
double altura=0.0;    // Para almacenar la altura del chasis en base según la odometria y la IMU
double posex=0.0;     // Y la X
double posey=0.0;     // Y la Y
double chasisx=0.44,chasisy=0.3,chasisz=0.07;
double x_deteccion_obstaculo=0.0;  // Punto en el que se detecta el obstaculo por primera vez
double x_inicio_obstaculo=1000.0;  // Punto de inicio del obstaculo
double x_fin_obstaculo=0.0;        // Punto de fin del obstaculo
double obstaculo_altura=0.0;       // altura del obstaculo
double radio=0.02;                 // radio de la rueda en metros
double distancia_inicio_obstaculo=1000.0; // Calculo de la distancia del eje de la rueda del brazo delantero al inicio
double distancia_fin_obstaculo=1000;
double distancia_chasis_inicio_obstaculo=1000.0; // Calculo de la distancia del eje de la rueda del brazo delantero al inicio
double distancia_chasis_fin_obstaculo=1000;
double principio_arriba=0.0;
bool segunda_fase_subida=false;
double chasis_z_suelo=0;
tf::TransformListener *tf_listener; 
pcl::PointCloud<pcl::PointXYZ>::Ptr ultimaentradaConvertida;
bool calculoCloudCompleta=false;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_completa (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_camino (new pcl::PointCloud<pcl::PointXYZRGB>);
bool enviar_mensaje=false;
double altura_base_rueda_delantera=0.0;
double altura_base_chasis=0.0;
double alto_chasis=0.015;
//tf::TransformListener listener= new tf::TransformListener();
  
double rosTimeToDouble(ros::Time RosTime) //ToDo implement in a library
{ 
  double a;
  double b;
  double c;
  a=RosTime.sec;
  b=RosTime.nsec;
  c=1.0*a + 1.0*b*10.0e-10;
  return c;
} 
void actualizaBrazoDelantero(double angulo)
{
    cmdBasicMovement_.arm_front_left = angulo+(3.14/360);
    cmdBasicMovement_.arm_front_right = angulo+(3.14/360);
}
void actualizaBrazoTrasero(double angulo)
{
    cmdBasicMovement_.arm_back_left = angulo+(3.14/360);
    cmdBasicMovement_.arm_back_right = angulo+(3.14/360);
}
void reset()
{
  obstaculo_altura=0.0;
}
void cloud_cb (const pcl::PCLPointCloud2ConstPtr& cloud_blob)
{
  int nuevoverde=0;
  // Se transforma la nube a odom
  pcl::PointCloud<pcl::PointXYZ>::Ptr entradaConvertida (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PointCloud<pcl::PointXYZ>::Ptr outcloud2 (new pcl::PointCloud<pcl::PointXYZ> ());
  ros::Time now = ros::Time::now();
  tf_listener->waitForTransform("/odom", (*cloud_blob).header.frame_id, now, ros::Duration(5.0));

  pcl::fromPCLPointCloud2(*cloud_blob, *entradaConvertida);
  pcl_ros::transformPointCloud("/odom",*entradaConvertida, *outcloud2, *tf_listener);
  pcl::PCLPointCloud2::Ptr cloud_transformada (new pcl::PCLPointCloud2);
  pcl::toPCLPointCloud2(*outcloud2,*cloud_transformada);
  // Como ha sido necesario realizar un cambio de formato, se vuele a cambiar para reducir el tamaño
  pcl::PCLPointCloud2::Ptr cloud_filtered_blob (new pcl::PCLPointCloud2);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_p (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);

  // Crear el objeto a filtrar, haciendo un downsample de la entrada usando un tamaño de hoja de 1 cm.
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  //sor.setInputCloud (cloud_blob);
  sor.setInputCloud (cloud_transformada);
  //std::cout << "PointCloud before filtering has: " << cloud_blob->points.size () << " data points." << std::endl; //*

  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  // El resultado lo introducims en cloud_filtered_blob
  sor.filter (*cloud_filtered_blob);

  // Convertimos la nube binaria, a un formato mas manejable en cloud_filtered
  pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  // Creamos el plano de segmentación
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  // Optional
  //seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);//SACMODEL_PARALLEL_PLANE);//SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.01);
  seg.setAxis(Eigen::Vector3f(1,1,0));
  //seg.setEpsAngle (0.01);  
  seg.setEpsAngle(  30.0f * (M_PI/180.0f) );
  seg.setInputCloud (cloud_filtered);

  Eigen::Vector4f centroid;
  
  pcl::compute3DCentroid(*cloud_filtered, centroid);

  std::cout << "The XYZ coordinates of the centroid are: ("
        << centroid[0] << ", "
        << centroid[1] << ", "
        << centroid[2] << ")." << std::endl;

  // Calculamos el numero de puntos.  
  int i = 0, nr_points = (int) cloud_filtered->points.size ();
  pcl::PointXYZRGB minPt;
  pcl::PointXYZRGB maxPt;
  pcl::getMinMax3D(*cloud_filtered,minPt,maxPt);
  std::cout << "Max x: " << maxPt.x << std::endl;
  std::cout << "Max y: " << maxPt.y << std::endl;
  std::cout << "Max z: " << maxPt.z << std::endl;
  std::cout << "Min x: " << minPt.x << std::endl;
  std::cout << "Min y: " << minPt.y << std::endl;
  std::cout << "Min z: " << minPt.z << std::endl;
  //obstaculo_altura=maxPt.y;

  //x_fin_obstaculo=maxPt.x;
  //altura=base-0.77;
  //altura=odometria->pose.pose.position.z;
  pcl::IndicesPtr remaining (new std::vector<int>);
  remaining->resize (nr_points);
  for (size_t i = 0; i < remaining->size (); ++i) { (*remaining)[i] = static_cast<int>(i); }

  // While 30% of the original cloud is still there
  while (remaining->size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setIndices (remaining);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0) break;

    // Extract the inliers
    std::vector<int>::iterator it = remaining->begin();
    for (size_t i = 0; i < inliers->indices.size (); ++i)
    {
      int curr = inliers->indices[i];
      // Remove it from further consideration.
      while (it != remaining->end() && *it < curr) { ++it; }
      if (it == remaining->end()) break;
      if (*it == curr) it = remaining->erase(it);
    }
    i++;
  }
  std::cout << "Encontrados " << i << " planos. Tenemos " << nr_points << std::endl;
  // Definimos el color verde
    uint8_t r = 0, g = 255, b = 0;
    uint8_t rojo_r = 255, rojo_g = 0, rojo_b = 0;
    uint32_t verde_rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
    uint32_t rojo_rgb = ((uint32_t)rojo_r << 16 | (uint32_t)rojo_g << 8 | (uint32_t)rojo_b);
    uint32_t azul_rgb = ((uint32_t)0 << 16 | (uint32_t)0 << 8 | (uint32_t)255);
    uint32_t negro_rgb = ((uint32_t)0 << 16 | (uint32_t)0 << 8 | (uint32_t)0);
    uint32_t blanco_rgb = ((uint32_t)255 << 16 | (uint32_t)255 << 8 | (uint32_t)255);
  // Color all the non-planar things.
    //
    pcl::PointXYZRGB referencia;
    referencia.y=0;
    referencia.rgb=*reinterpret_cast<float*>(&azul_rgb);
    double z_max=1;
  for (std::vector<int>::iterator it = remaining->begin(); it != remaining->end(); ++it)
  {
//std::cout << "x " << cloud_filtered->at(*it).x << "  y "<< cloud_filtered->at(*it).y << "  z " << cloud_filtered->at(*it).z << "  z" << std::endl;
    if (cloud_filtered->at(*it).z>z_max)
    {
        cloud_filtered->at(*it).rgb = *reinterpret_cast<float*>(&rojo_rgb);
    }/*
    else if(cloud_filtered->at(*it).y<0.1)
    {
        cloud_filtered->at(*it).rgb = *reinterpret_cast<float*>(&rojo_rgb);
    }*/
    else
    {
        cloud_filtered->at(*it).rgb = *reinterpret_cast<float*>(&verde_rgb);
        nuevoverde++;
        if (cloud_filtered->at(*it).z>0.01 && cloud_filtered->at(*it).x<x_inicio_obstaculo) //x_inicio_obstaculo<0.1 && 
        {
          x_inicio_obstaculo=cloud_filtered->at(*it).x;
          x_fin_obstaculo=x_inicio_obstaculo+1;
        }
    }
    if (cloud_filtered->at(*it).z>obstaculo_altura)
    {
          obstaculo_altura=cloud_filtered->at(*it).z;
    } 
    if (cloud_filtered->at(*it).z<z_max && (cloud_filtered->at(*it).y<0.001 || cloud_filtered->at(*it).y>-0.001 ))
    {
        referencia.x=cloud_filtered->at(*it).x; 
        //referencia.y=cloud_filtered->at(*it).y;
        referencia.y=0;
        
        // Si la Z de este punto es mayor que la attual, sta será la nueva altura del obstaculo
   
        cloud_filtered->push_back(referencia);
        cloud_camino->push_back(referencia);
    }


  }
  // Agregamos z en azul.
  // z-azul
  pcl::PointXYZRGB o;
  o.rgb=*reinterpret_cast<float*>(&azul_rgb);
  double punto=0.0;
  for (int i=0;i<1000;i++)
  {

    o.x = 0;
    o.y = 0;
    o.z = punto;
    punto=punto+0.001;
    cloud_filtered->push_back(o);
  }
  // y-verde
  o.rgb=*reinterpret_cast<float*>(&verde_rgb);
  punto=0.0;
  for (int i=0;i<1000;i++)
  {

    o.x = 0;
    o.y = punto;
    o.z = 0;
    punto=punto+0.001;
    cloud_filtered->push_back(o);
  }
  // x-rojo
  o.rgb=*reinterpret_cast<float*>(&rojo_rgb);
  punto=0.0;
  for (int i=0;i<1000;i++)
  {

    o.x = punto;
    o.y = 0;
    o.z = 0;
    punto=punto+0.001;
    cloud_filtered->push_back(o);
  }
// Guia del camino en negro
  o.rgb=*reinterpret_cast<float*>(&negro_rgb);
  punto=0.0;
  double punto_fijo_x=0.2;
  double punto_fijo_y=0.25;
  double punto_fijo_z=0;
  o.x = punto_fijo_x;
  o.z = punto_fijo_z;
  // Y fija en 0.353308
  for (int i=0;i<1000;i++)
  {
    o.x = punto;
    o.y = punto_fijo_y;

    cloud_filtered->push_back(o);
    o.y = -punto_fijo_y;

    cloud_filtered->push_back(o);
 
    punto=punto+0.01;

  }

  // Publish the planes we found.
  verdes=nuevoverde;
  
  
  pub.publish (cloud_filtered);
  pub3.publish (cloud_camino);
  //pub4.publish (cloud_completa);
}
 void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
 {
   ROS_INFO("pose.z: [%f]  ",msg->pose.pose.position.z);
   altura=msg->pose.pose.position.z;
   posex=msg->pose.pose.position.x;
   posey=msg->pose.pose.position.y;
   //geometry_msgs::PointStamped rueda_delantera_izquierda;
   //listener.transformPoint("base_link", laser_point, base_point);
  
 }
 void linkCallback(const gazebo_msgs::LinkStates::ConstPtr& msg)
 {
  actualTime = rosTimeToDouble( ros::Time::now());
  int referencia_delantera=14,referencia_trasera=9,referencia_chasis=7;



  if((actualTime-lastLoopTime) >= 1.05 ){
    
   ROS_INFO("link: chasis x [%f] y [%f] z [%f]  ",msg->pose[referencia_chasis].position.x,msg->pose[referencia_chasis].position.y,msg->pose[referencia_chasis].position.z);
   ROS_INFO("link: front wheel x [%f] y [%f] z [%f]  ",msg->pose[referencia_delantera].position.x,msg->pose[referencia_delantera].position.y,msg->pose[referencia_delantera].position.z);   
   ROS_INFO("link: back wheel x [%f] y [%f] z [%f]  ",msg->pose[referencia_trasera].position.x,msg->pose[referencia_trasera].position.y,msg->pose[referencia_trasera].position.z);   //altura=msg->pose.pose.position.z;
   ROS_INFO("link: obstaculo x [%f] y [%f] z [%f]  ",msg->pose[6].position.x,msg->pose[6].position.y,msg->pose[6].position.z);//posex=msg->pose.pose.position.x;
   ROS_INFO("Verdes: [%d] Ultimo trasero: [%f]  Ultimo delantero: [%f] Altura: [%f] Pose x: [%f] Pose y: [%f] Principio_arriba: [%f]",verdes,ultimo_angulo,ultimo_angulo_delantero,altura,posex,posey,principio_arriba);
   ROS_INFO("Obstaculo distancia_inicio: [%f] distancia_fin: [%f] x_inicio: [%f], x_fin_obstaculo: [%f],  x_deteccion_obstaculo: [%f], altura: [%f]", distancia_inicio_obstaculo,distancia_fin_obstaculo,x_inicio_obstaculo,x_fin_obstaculo,x_deteccion_obstaculo,obstaculo_altura);
   std::cout << "Estado: " << estado << " \n";
   //ROS_INFO(estado);
   lastLoopTime = actualTime;
   if (chasis_z_suelo==0)
    {
      chasis_z_suelo=msg->pose[7].position.z+0.01;
    }
  }
  distancia_inicio_obstaculo=x_inicio_obstaculo-msg->pose[referencia_delantera].position.x;
  distancia_fin_obstaculo=x_fin_obstaculo-msg->pose[referencia_delantera].position.x;
  distancia_chasis_inicio_obstaculo=x_inicio_obstaculo-msg->pose[referencia_chasis].position.x;
  distancia_chasis_fin_obstaculo=x_fin_obstaculo-msg->pose[referencia_chasis].position.x;
  cmdBasicMovement_.robot_x = 0;
    //cmdBasicMovement_.robot_y = 0;
    //cmdBasicMovement_.robot_zrot = 0;
    actualizaBrazoDelantero(ultimo_angulo_delantero);
    actualizaBrazoTrasero(ultimo_angulo);

  switch(estado)
  {
    case inicio:
      if (enviar_mensaje)
        {
          std::cout << "inicio de proceso\n";
          enviar_mensaje=false;
        } 
      break;
    case suelo  : 
      cmdBasicMovement_.robot_x = 0.5;
      if (verdes>300)
      {
        x_deteccion_obstaculo=msg->pose[7].position.x;
        
        
      }
      if (distancia_inicio_obstaculo<0.2)
      {
        estado=subiendo;
        enviar_mensaje=true;
      }
      //std::cout << "suelo\n";
      break;
    case subiendo:
      cmdBasicMovement_.robot_x = 0.3;
      altura_base_rueda_delantera=msg->pose[referencia_delantera].position.z-radio;
      altura_base_chasis=msg->pose[7].position.z-(alto_chasis);

      std::cout << "subiendo" << altura_base_rueda_delantera << " Altura obstaculo "<< (obstaculo_altura) << " distancia "<< distancia_inicio_obstaculo << " altura base chasis " << altura_base_chasis << "\n";
      //Subimos la rueda delantera
      if(distancia_inicio_obstaculo<0.2 && altura_base_rueda_delantera<(obstaculo_altura))
      {
        std::cout << "subimos rueda" << altura_base_rueda_delantera << "\n";
        if (enviar_mensaje)
        {
          std::cout << "subimos rueda" << altura_base_rueda_delantera << "\n";
          enviar_mensaje=false;
        } 
        ultimo_angulo_delantero=ultimo_angulo_delantero-0.002;

      }
      // Acercamos el robot

      else if(altura_base_rueda_delantera>(obstaculo_altura) && distancia_inicio_obstaculo>0)
      {
        std::cout << "Acercamos el robot\n";
        cmdBasicMovement_.robot_x = 0.5;
      }
      //Si el chasis esta arriba
      else if(altura_base_chasis>=(obstaculo_altura))//  && distancia_inicio_obstaculo>-0.1)
      {
        std::cout << "andamos por arriba\n";
        cmdBasicMovement_.robot_x = 0.5;
        estado=arriba;
        //ultimo_angulo_delantero=ultimo_angulo_delantero+0.002;
      }//Subimos la plataforma
      else if(altura_base_chasis<(obstaculo_altura))
      {
        std::cout << "subimos el chasis\n";
        cmdBasicMovement_.robot_x = 0.1;
        ultimo_angulo_delantero=ultimo_angulo_delantero+0.0002;
      }
      else
        std::cout << "subiendo: ninguna condición\n";
      break;
      
    case arriba :
      cmdBasicMovement_.robot_x = 0.1;

      if(distancia_chasis_inicio_obstaculo>0)
      {
        std::cout << "andamos por arriba - inicio " << distancia_chasis_inicio_obstaculo << "\n";
        cmdBasicMovement_.robot_x = 0.50;
        //ultimo_angulo_delantero=ultimo_angulo_delantero-0.002;

      }
      else if(distancia_chasis_fin_obstaculo<0.1)
      {
        std::cout << "bajando estado arriba" << distancia_fin_obstaculo << "\n";
        estado=bajando;
        //cmdBasicMovement_.robot_x = 0.20;
        ultimo_angulo_delantero=ultimo_angulo_delantero+0.001;
        ultimo_angulo=0;
      }
      //else if(distancia_fin_obstaculo<0.2)
      else if(distancia_chasis_inicio_obstaculo<0 && ultimo_angulo_delantero>0)
      {
        std::cout << "andamos por arriba -cerca del fin  " << distancia_chasis_inicio_obstaculo << "\n";
        //cmdBasicMovement_.robot_x = 0.20;
        ultimo_angulo_delantero=ultimo_angulo_delantero-0.001;
        
        cmdBasicMovement_.robot_x = 0.50;
      }
      else if(distancia_chasis_inicio_obstaculo<-(chasisx/2) && ultimo_angulo<0)
      {
        ultimo_angulo=ultimo_angulo+0.001;
      }
      else if(distancia_chasis_inicio_obstaculo<-(chasisx/2))
      {
        cmdBasicMovement_.robot_x = 0.50;
      }  
      else
      {
        std::cout << "arriba: ninguna condición " << distancia_chasis_inicio_obstaculo << " - " << distancia_chasis_fin_obstaculo <<"\n";
      }
      if(ultimo_angulo==0)
      {
        cmdBasicMovement_.robot_x = 0.20;
      }
      break;
      
    case bajando :
      cmdBasicMovement_.robot_x = 0;
      if(msg->pose[7].position.z<chasis_z_suelo && distancia_fin_obstaculo<-1.6)
      {
        std::cout << "en el suelo, lejos" << distancia_fin_obstaculo << "\n";
        estado=inicio;
        enviar_mensaje=true;
        ultimo_angulo_delantero=0;
        ultimo_angulo=0;
      }
      else if(msg->pose[7].position.z<chasis_z_suelo && distancia_fin_obstaculo<-0.9)
      {
        std::cout << "en el suelo, alejandonos" << distancia_fin_obstaculo << "\n";
        ultimo_angulo_delantero=0;
        cmdBasicMovement_.robot_x = 0.20;
        //ultimo_angulo=0;
      }
      else if(distancia_fin_obstaculo<-1)
      {
        std::cout << "bajando chasis" << distancia_fin_obstaculo << "\n";
        ultimo_angulo_delantero=ultimo_angulo_delantero-0.0001;
        cmdBasicMovement_.robot_x = 0.05;
        //ultimo_angulo=0;        
      }      
      else if(distancia_fin_obstaculo<-0.3)
      {
        std::cout << "bajando rueda delantera" << distancia_fin_obstaculo << "\n";
        ultimo_angulo_delantero=ultimo_angulo_delantero+0.001;
        cmdBasicMovement_.robot_x = 0.20;
        //ultimo_angulo=0;        
      }
      else if(distancia_fin_obstaculo<-0.1)
      {

        std::cout << "bajando estabilizando trasera" << distancia_fin_obstaculo << "\n";
        ultimo_angulo_delantero=ultimo_angulo_delantero+0.001;
        ultimo_angulo=0;
        //cmdBasicMovement_.robot_x = 0.20;
      }
      if(ultimo_angulo==0)
      {
        cmdBasicMovement_.robot_x = 0.20;
      }
      break;
  }

  if(ultimo_angulo_delantero<-1.2)
  {
    ultimo_angulo_delantero=-1.2;
  }
  if(ultimo_angulo_delantero>1.2)
  {
    ultimo_angulo_delantero=1.2;
  }
  if(ultimo_angulo<-1.2)
  {
    ultimo_angulo=-1.2;
  }
  if(ultimo_angulo>1.2)
  {
    ultimo_angulo=1.2;
  }
//  cmdBasicMovement_.left_back_leg = ultimo_angulo;//+(3.14/360);
//  cmdBasicMovement_.left_front_leg=ultimo_angulo_delantero;//+(3.14/360);

    actualizaBrazoDelantero(ultimo_angulo_delantero);
    actualizaBrazoTrasero(ultimo_angulo);


  pub2.publish(cmdBasicMovement_);
  
 }
 void chatterCallback(const sensor_msgs::Imu::ConstPtr& msg)
 {
  
ROS_INFO("Imu Orientation x: [%f], y: [%f], z: [%f], w: [%f]", msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);
    actualizaBrazoDelantero(ultimo_angulo_delantero);
    actualizaBrazoTrasero(ultimo_angulo);

  if (msg->orientation.y < 0.01 && msg->orientation.y>-0.01)
    {
     // ROS_INFO("No hay inclinación");
      //cmdBasicMovement_.robot_x=.5;

      

      //pub2.publish(cmdBasicMovement_);
    }
    else if(msg->orientation.y>0.05)
    {
            ultimo_angulo=ultimo_angulo+0.05;
            actualizaBrazoTrasero(ultimo_angulo);   
    }
    else if(msg->orientation.y<-0.05)
    {
          ultimo_angulo=ultimo_angulo-0.05;
          actualizaBrazoTrasero(ultimo_angulo);   
    }
    else if(msg->orientation.y>0.01)
    {
            ultimo_angulo=ultimo_angulo+0.01;
            actualizaBrazoTrasero(ultimo_angulo);   

    }
    else if(msg->orientation.y<-0.01)
    {
          ultimo_angulo=ultimo_angulo-0.01;
          actualizaBrazoTrasero(ultimo_angulo);  
    }
    else
    {
      ROS_INFO("En el else");

    }
    //cmdBasicMovement_.robot_x=.0;
  pub2.publish(cmdBasicMovement_);

}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "planes");
  ros::NodeHandle nh;
 tf_listener= new tf::TransformListener();
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);
  ros::Subscriber sub2 = nh.subscribe("/mybot/imu/chassis", 1000, chatterCallback);
  //ros::Subscriber sub3 = nh.subscribe("/odom", 1000, odomCallback);
  ros::Subscriber sub4 = nh.subscribe("/gazebo/link_states", 1000, linkCallback);
  
  // Create a ROS publisher for the output point cloud
pub = nh.advertise<pcl::PCLPointCloud2> ("planes", 1);
pub2 = nh.advertise<cpmbot::cpmbot_basic>("basic/command", 10);
pub3 = nh.advertise<pcl::PCLPointCloud2> ("camino", 2);
//pub4 = nh.advertise<pcl::PCLPointCloud2> ("planesTransformado", 3);

  // Spin
  ros::spin ();
}

