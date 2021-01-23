# Este repositorio es para la reconstrucción del entorno con 4 cámaras
# El dataset implementado fue obtenido gracias a los datasets libres de VCL (Visual Computing Lab) - http://vcl.iti.gr/dataset/datasets-of-multiple-kinect2-rgb-d-streams-and-skeleton-tracking/
El cómo obtuvieron las imágenes es explicado en: An integrated platform for live 3D human.pdf, algunas de estas imágenes se encuentra en el directorio *data*


## 1

Se corre *rosrun reconstruccion_pkg big_gen_topic*. El archivo .py se encuentra en el direcotrio */scripts* este nodo genera los topics relacionados con las imágenes rgb, depth y de la información de los parámetros intrínsecos de estas cámaras.

## 2 

-> Se corre:

*rosrun nodelet nodelet manager  __name:=nodelet_manager*

-> Se corre: 

rosrun nodelet nodelet load depth_image_proc/register /depth_image_proc_register:=depth_image_proc_register_d1 rgb/camera_info:=/rgb/camera_info_d1 depth/camera_info:=/depth/camera_info_d1 depth/image_rect:=/depth/image_rect_d1 depth_registered/camera_info:=depth_registerd/camera_info_d1 depth_registered/image_rect:=depth_registered/image_rect_d1 nodelet_manager

rosrun nodelet nodelet load depth_image_proc/register /depth_image_proc_register:=depth_image_proc_register_d3 rgb/camera_info:=/rgb/camera_info_d3 depth/camera_info:=/depth/camera_info_d3 depth/image_rect:=/depth/image_rect_d3 depth_registered/camera_info:=depth_registerd/camera_info_d3 depth_registered/image_rect:=depth_registered/image_rect_d3 nodelet_manager

rosrun nodelet nodelet load depth_image_proc/register /depth_image_proc_register:=depth_image_proc_register_d4 rgb/camera_info:=/rgb/camera_info_d4 depth/camera_info:=/depth/camera_info_d4 depth/image_rect:=/depth/image_rect_d4 depth_registered/camera_info:=depth_registerd/camera_info_d4 depth_registered/image_rect:=depth_registered/image_rect_d4 nodelet_manager

rosrun nodelet nodelet load depth_image_proc/register /depth_image_proc_register:=depth_image_proc_register_d5 rgb/camera_info:=/rgb/camera_info_d5 depth/camera_info:=/depth/camera_info_d5 depth/image_rect:=/depth/image_rect_d5 depth_registered/camera_info:=depth_registerd/camera_info_d5 depth_registered/image_rect:=depth_registered/image_rect_d5 nodelet_manager

**Esto relaciona la imágenes de profundidad con las imágenes de color de cada cámara para generar posteriormente la nube de puntos.**

-> Se corre: 

rosrun nodelet nodelet load depth_image_proc/point_cloud_xyzrgb __name:=d1_node rgb/camera_info:=rgb/camera_info_d1 rgb/image_rect_color:=rgb/image_rect_color_d1 depth_registered/image_rect:=depth_registered/image_rect_d1 depth_registered/points:=depth_registerd/points_d1 nodelet_manager

rosrun nodelet nodelet load depth_image_proc/point_cloud_xyzrgb __name:=d3_node rgb/camera_info:=rgb/camera_info_d3 rgb/image_rect_color:=rgb/image_rect_color_d3 depth_registered/image_rect:=depth_registered/image_rect_d3 depth_registered/points:=depth_registerd/points_d3 nodelet_manager

rosrun nodelet nodelet load depth_image_proc/point_cloud_xyzrgb __name:=d4_node rgb/camera_info:=rgb/camera_info_d4 rgb/image_rect_color:=rgb/image_rect_color_d4 depth_registered/image_rect:=depth_registered/image_rect_d4 depth_registered/points:=depth_registerd/points_d4 nodelet_manager

rosrun nodelet nodelet load depth_image_proc/point_cloud_xyzrgb __name:=d5_node rgb/camera_info:=rgb/camera_info_d5 rgb/image_rect_color:=rgb/image_rect_color_d5 depth_registered/image_rect:=depth_registered/image_rect_d5 depth_registered/points:=depth_registerd/points_d5 nodelet_manager


**Esto genera las distintas nube de puntos para cada una de las cámaras.**

## 3

Se corre: 

rosrun tf2_ros static_transform_publisher 0.01 0.05 0 0 0 -0.15 map d1_frame

rosrun tf2_ros static_transform_publisher 0.03 0.15 7.3 -0.045 -3.15 -0.157 map d3_frame

rosrun tf2_ros static_transform_publisher -3.60 0.2 3.63 0.1 1.605 -0.07 map d4_frame

rosrun tf2_ros static_transform_publisher 3.75 0 3.7 0 -1.55 -0.16 map d5_frame

**Se relaciona la posición de cada una de las cámaras respecto a un marco de referancia global (map)**

## 4 

Se visualizan las nubes de puntos en Rviz

## 5 

Se corre *rosrun reconstruccion_pkg filter_point_cloud* que permite visualizar los puntos que se encuentran en un área de trabajo de aproximadamente 2m^2, además se remueve los puntos pertencientes al suelo. 

## 6 

Se corre *rosrun reconstruccion_pkg concatenate_point_cloud* se concatenan las nubes de puntos de las distintas cámaras en una única nube de puntos

## Resultado 

![Screenshot from 2021-01-17 13-33-08](https://user-images.githubusercontent.com/77637361/105614516-e64a3580-5d97-11eb-8624-96f49e2b628a.png)




