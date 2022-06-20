#pragma once

#include <UHEMesh/HEMesh.h>

#include <UGM/UGM.h>

struct Vertex;
struct Edge;
struct Triangle;
struct HalfEdge;

//半边网格结构<顶点，边，面，另外半边>
using HEMeshXTraits = Ubpa::HEMeshTraits<Vertex, Edge, Triangle, HalfEdge>;

//顶点结构
struct Vertex : Ubpa::TVertex<HEMeshXTraits> {
	// you can add any attributes and mothods to Vertex
	Ubpa::pointf3 position{ 0.f };//当前坐标
	Ubpa::pointf3 old_position{ 0.f };//更新前坐标
	Ubpa::normalf normal{ 0.f };//顶点法向
};


//边结构
struct Edge : Ubpa::TEdge<HEMeshXTraits> {
	// you can add any attributes and mothods to Edge

	// [example]

	// Ubpa::pointf3 Midpoint() const {
	//     auto* p = HalfEdge()->Origin();
    //     auto* q = HalfEdge()->End();
	//     return Ubpa::pointf3::combine(std::array{ p,q }, 0.5f);
	// }
};

//半边结构
struct HalfEdge : Ubpa::THalfEdge<HEMeshXTraits> {
	// you can add any attributes and mothods to HalfEdge

};

//三角网格结构
struct Triangle : Ubpa::TPolygon<HEMeshXTraits> {
	// you can add any attributes and mothods to Triangle

	// [example]
	// 
	 float area{ 0.f };
	 
	 //确保为三角形
	 bool IsTriangle() const {
	     return Degree() == 3;
	 }
	 
	 float UpdateArea() {
		 //更新三角网格面积
	     assert(IsTriangle());
	     auto* p0 = HalfEdge()->Origin();
	     auto* p1 = HalfEdge()->Next()->Origin();
	     auto* p2 = HalfEdge()->Next()->Next()->Origin();
		 auto d01 = p1->old_position - p0->old_position;
		 auto d02 = p2->old_position - p0->old_position;
		 area = 0.5f * d02.cross(d01).norm();
	     return area;
	 }
};


//半边网格
struct HEMeshX : Ubpa::HEMesh<HEMeshXTraits> {
	// you can add any attributes and mothods to HEMeshX
};
