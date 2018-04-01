#pragma once


//这个类虽然短，但是对于地图搭建非常方便
class MapBuild
{
public:
	//rows是每行aruco码的数量，cols是行数（或者理解为每列aruco码的数量），space是码和码中心点之间的真实间隔
#define rows 10
#define cols 10
#define space 1.0
	int row = 0;
	int col = 0;

public:
	//************two functions of translating id to location*************//
double idTranx(int id)
{
	row = id % cols;
	return (row*space);
}
double idTrany(int id)
{
	col = id / cols;
	return (col*space);
}

};













