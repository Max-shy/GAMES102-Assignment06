#pragma once

#include <UHEMesh/HEMesh.h>

#include <UGM/UGM.h>

struct Vertex;
struct Edge;
struct Triangle;
struct HalfEdge;

//�������ṹ<���㣬�ߣ��棬������>
using HEMeshXTraits = Ubpa::HEMeshTraits<Vertex, Edge, Triangle, HalfEdge>;

//����ṹ
struct Vertex : Ubpa::TVertex<HEMeshXTraits> {
	// you can add any attributes and mothods to Vertex
	Ubpa::pointf3 position{ 0.f };//��ǰ����
	Ubpa::pointf3 old_position{ 0.f };//����ǰ����
	Ubpa::normalf normal{ 0.f };//���㷨��
};


//�߽ṹ
struct Edge : Ubpa::TEdge<HEMeshXTraits> {
	// you can add any attributes and mothods to Edge

	// [example]

	// Ubpa::pointf3 Midpoint() const {
	//     auto* p = HalfEdge()->Origin();
    //     auto* q = HalfEdge()->End();
	//     return Ubpa::pointf3::combine(std::array{ p,q }, 0.5f);
	// }
};

//��߽ṹ
struct HalfEdge : Ubpa::THalfEdge<HEMeshXTraits> {
	// you can add any attributes and mothods to HalfEdge

};

//��������ṹ
struct Triangle : Ubpa::TPolygon<HEMeshXTraits> {
	// you can add any attributes and mothods to Triangle

	// [example]
	// 
	 float area{ 0.f };
	 
	 //ȷ��Ϊ������
	 bool IsTriangle() const {
	     return Degree() == 3;
	 }
	 
	 float UpdateArea() {
		 //���������������
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


//�������
struct HEMeshX : Ubpa::HEMesh<HEMeshXTraits> {
	// you can add any attributes and mothods to HEMeshX
};
