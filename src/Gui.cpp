#include "Gui.hpp"

GUI::GUI(int rows, int cols){
	display_size=cv::Size(rows, cols);
	display=Mat(display_size, CV_8UC3, Scalar(220,220,220));
}

void GUI::drawCar(State src){
	RotatedRect rotRect = RotatedRect(Point2f(src.x*display_size.width/MAPX, src.y*display_size.height/MAPY), Size2f(BOT_L*display_size.width/MAPX, BOT_W*display_size.height/MAPY), src.theta*Theta_Res);
	Point2f vert[4];
	rotRect.points(vert);
	for(int i=0;i<4;i++)
		line(display, vert[i], vert[(i+1)%4], Scalar(200, 0, 0));

	circle(display, Point2f((src.x + 17*cos(src.theta*2.0*PI/Theta))*display_size.width/MAPX, (src.y + 17*sin(src.theta*2.0*PI/Theta))*display_size.width/MAPX), 5, Scalar(255, 0, 0));
}

void GUI::drawObs(Map map){
	for(int i=0;i<MAPX;i++)
		for(int j=0;j<MAPY;j++)
			if(map.obs_map[i][j]){

				for(int k=i*display_size.width/MAPX;k<(i+1)*display_size.width/MAPX;k++)
					for(int l=j*display_size.height/MAPY;l<(j+1)*display_size.height/MAPY;l++){
						display.at<Vec3b>(l,k)={128, 128, 128};
					}
			}
}

void GUI::markPoint(int i, int j){
	if(i<0 || i>MAPX || j<0 || j>MAPY)
		return;

	for(int k=i*display_size.width/MAPX;k<(i+1)*display_size.width/MAPX;k++)
		for(int l=j*display_size.height/MAPY;l<(j+1)*display_size.height/MAPY;l++){
						display.at<Vec3b>(k,l)={0, 255, 0};
					}
}

void GUI::show(int t){
	Mat temp=display;
	//resize(display, temp, display_size);
	imshow("Display", temp);
	waitKey(t);
	return;
}
