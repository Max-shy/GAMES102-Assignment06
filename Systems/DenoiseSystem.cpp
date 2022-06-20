#include "DenoiseSystem.h"

#include "../Components/DenoiseData.h"
#include <_deps/imgui/imgui.h>
#include <spdlog/spdlog.h>
#include<cmath>
#include<algorithm>

using namespace Ubpa;

void DenoiseSystem::OnUpdate(Ubpa::UECS::Schedule& schedule) {
	schedule.RegisterCommand([](Ubpa::UECS::World* w) {
		auto data = w->entityMngr.GetSingleton<DenoiseData>();
		if (!data)
			return;

		if (ImGui::Begin("Denoise")) {
			if (ImGui::Button("Mesh to HEMesh")) {
				//转换数据结构，将网格转换为半边网格
				data->heMesh->Clear();
				[&]() {
					if (!data->mesh) {
						spdlog::warn("mesh is nullptr");
						return;
					}

					if (data->mesh->GetSubMeshes().size() != 1) {
						spdlog::warn("number of submeshes isn't 1");
						return;
					}

					data->copy = *data->mesh;
					//获取顶点数据
					std::vector<size_t> indices(data->mesh->GetIndices().begin(), data->mesh->GetIndices().end());
					data->heMesh->Init(indices, 3);//为三角形网格

					//获取顶点法向量
					const auto& normals = data->mesh->GetNormals();
					assert(normals.size() == data->heMesh->Vertices().size());

					//是否为三角形网格结构
					if (!data->heMesh->IsTriMesh())
						spdlog::warn("HEMesh init fail");
					
					//赋值半边网格结构的各个顶点坐标、法向量
					for (size_t i = 0; i < data->mesh->GetPositions().size(); i++) {
						data->heMesh->Vertices().at(i)->position = data->mesh->GetPositions().at(i);
						data->heMesh->Vertices().at(i)->normal = normals[i];
					}

					spdlog::info("Mesh to HEMesh success");
				}();
			}

			//进行极小曲面法迭代
			if (ImGui::Button("Add Noise")) {
				[&]() {
					//判断是否为三角形半边网格结构
					if (!data->heMesh->IsTriMesh()) {
						spdlog::warn("HEMesh isn't triangle mesh");
						return;
					}

					
					for (int i = 0; i < data->iter_time; ++i) {
						//迭代iter_time次
						for (auto* v : data->heMesh->Vertices()) {
							//对半边网格数据的每个顶点
							//将当前顶点坐标，作为下次迭代的上次坐标（更新顶点旧坐标）
							v->old_position = v->position;
						}

						for (auto* v : data->heMesh->Vertices()) {
							//对半边网格数据的每个顶点
							//固定边界顶点,如果为边界顶点，不更新
							if (v->IsOnBoundary())
								continue;

							const auto P = v->old_position;//当前顶点坐标
							valf3 Hn = { 0.f };//当前顶点的平均曲率
							
							//计算每个更新后每个三角形面积的和（曲面总面积）
							float A = 0.f;
							const auto& adj_a = v->AdjPolygons();//获取每个三角形网格
							for (auto* poly : adj_a) {
								//面积求和
								A += poly->UpdateArea();
							}

							//输出当前曲面总面积
							spdlog::info(std::to_string(A));
							assert(A > 0.f);//确保总面积>0

							//更新当前顶点经过的所有三角形网格的顶点坐标
							const auto& adj_v = v->AdjVertices();
							for (size_t i = 0; i < adj_v.size(); ++i) {
								//获取当前三角形网格的顶点的坐标
								const auto p0 = (i != 0 ? adj_v[i - 1]->old_position : adj_v.back()->old_position);
								const auto p1 = adj_v[i]->old_position;
								const auto p2 = (i + 1 < adj_v.size() ? adj_v[i + 1]->old_position : adj_v[0]->old_position);

								float cos_a = (p1 - p0).dot(P - p0);
								float cos_b = (p1 - p2).dot(P - p2);
								cos_a = std::min(cos_a, 0.9f);
								cos_b = std::min(cos_b, 0.9f);
								float cot_a = cos_a / sqrt(1 - cos_a * cos_a);
								float cot_b = cos_b / sqrt(1 - cos_b * cos_b);
								//当前顶点所在面的曲率K(xi)（求和）
								Hn -= (cot_a + cot_b) * (P - p1);
							}
							//当前顶点的平均曲率 Hn' = 1/(4A) * (cot_aj + cot_bj) * (P - pi)
							Hn /= (4.0f * A);
							v->position = v->old_position + data->Lambda * Hn;//更新顶点坐标，继续迭代
							v->normal = Hn.normalize();//法向量归一化
						}

					}
					/*for (auto* v : data->heMesh->Vertices()) {
						v->position += data->Lambda * (
							2.f * Ubpa::vecf3{ Ubpa::rand01<float>(),Ubpa::rand01<float>() ,Ubpa::rand01<float>() } - Ubpa::vecf3{ 1.f }
						);
					}*/

					spdlog::info("Add noise success");
				}();
			}

			//对顶点进行法线贴图
			if (ImGui::Button("Set Normal to Color")) {
				[&]() {
					if (!data->mesh) {
						spdlog::warn("mesh is nullptr");
						return;
					}

					data->mesh->SetToEditable();
					const auto& normals = data->mesh->GetNormals();
					std::vector<rgbf> colors;
					for (const auto& n : normals)
						colors.push_back((n.as<valf3>() + valf3{ 1.f }) / 2.f);
					data->mesh->SetColors(std::move(colors));

					spdlog::info("Set Normal to Color Success");
				}();
			}

			//将半边网格结构，转化为网格结构
			if (ImGui::Button("HEMesh to Mesh")) {
				[&]() {
					if (!data->mesh) {
						spdlog::warn("mesh is nullptr");
						return;
					}

					if (!data->heMesh->IsTriMesh() || data->heMesh->IsEmpty()) {
						spdlog::warn("HEMesh isn't triangle mesh or is empty");
						return;
					}

					data->mesh->SetToEditable();

					const size_t N = data->heMesh->Vertices().size();
					const size_t M = data->heMesh->Polygons().size();
					std::vector<Ubpa::pointf3> positions(N);
					std::vector<uint32_t> indices(M * 3);
					for (size_t i = 0; i < N; i++)
						positions[i] = data->heMesh->Vertices().at(i)->position;
					for (size_t i = 0; i < M; i++) {
						auto tri = data->heMesh->Indices(data->heMesh->Polygons().at(i));
						indices[3 * i + 0] = static_cast<uint32_t>(tri[0]);
						indices[3 * i + 1] = static_cast<uint32_t>(tri[1]);
						indices[3 * i + 2] = static_cast<uint32_t>(tri[2]);
					}
					data->mesh->SetColors({});
					data->mesh->SetUV({});
					data->mesh->SetPositions(std::move(positions));
					data->mesh->SetIndices(std::move(indices));
					data->mesh->SetSubMeshCount(1);
					data->mesh->SetSubMesh(0, { 0, M * 3 });
					data->mesh->GenUV();
					data->mesh->GenNormals();
					data->mesh->GenTangents();

					spdlog::info("HEMesh to Mesh success");
				}();
			}

			//覆盖网格
			if (ImGui::Button("Recover Mesh")) {
				[&]() {
					if (!data->mesh) {
						spdlog::warn("mesh is nullptr");
						return;
					}
					if (data->copy.GetPositions().empty()) {
						spdlog::warn("copied mesh is empty");
						return;
					}

					*data->mesh = data->copy;

					spdlog::info("recover success");
				}();
			}
		}
		ImGui::End();
	});
}

