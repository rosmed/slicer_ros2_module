#include "vtkAssImpConversion.h"

#include <vtkAppendPolyData.h>
#include <vtkCellArray.h>
#include <vtkFloatArray.h>
#include <vtkPointData.h>
#include <vtkPoints.h>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <assimp/config.h>

#include <algorithm>
#include <iostream>

std::vector<vtkAssImpMeshPart> vtkAssImpConversion::vtkAssImpToPolyDataParts(const std::string& filename)
{
  Assimp::Importer importer;
  // URDF meshes are expected in their native coordinate frame.
  // Prevent Assimp from rotating Collada/DAE meshes to convert
  // from Z_UP to Y_UP, which would introduce a 90-degree rotation.
  importer.SetPropertyBool(AI_CONFIG_IMPORT_COLLADA_IGNORE_UP_DIRECTION, true);
  const aiScene* scene = importer.ReadFile(filename,
                                           aiProcess_Triangulate |
                                           aiProcess_GenSmoothNormals |
                                           aiProcess_JoinIdenticalVertices |
                                           aiProcess_PreTransformVertices |
                                           aiProcess_FixInfacingNormals);

  if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
    std::cerr << "Assimp failed to read file: " << filename << " (" << importer.GetErrorString() << ")" << std::endl;
    return {};
  }

  auto resolveMeshColor = [&](aiMesh* mesh) {
    std::vector<double> meshColor{0.5, 0.5, 0.5};
    if (!mesh || mesh->mMaterialIndex >= scene->mNumMaterials) {
      return meshColor;
    }
    aiMaterial* mat = scene->mMaterials[mesh->mMaterialIndex];
    aiColor4D diffuse;
    if (AI_SUCCESS == aiGetMaterialColor(mat, AI_MATKEY_COLOR_DIFFUSE, &diffuse)) {
      const double r = static_cast<double>(diffuse.r);
      const double g = static_cast<double>(diffuse.g);
      const double b = static_cast<double>(diffuse.b);
      const double maxChannel = std::max(r, std::max(g, b));
      if (maxChannel > 0.02) {
        meshColor = {r, g, b};
      }
    }
    return meshColor;
  };

  std::vector<vtkAssImpMeshPart> parts;
  parts.reserve(scene->mNumMeshes);

  for (unsigned int meshIndex = 0; meshIndex < scene->mNumMeshes; ++meshIndex) {
    aiMesh* mesh = scene->mMeshes[meshIndex];
    if (!mesh || mesh->mNumVertices == 0) {
      continue;
    }

    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkCellArray> polys = vtkSmartPointer<vtkCellArray>::New();
    vtkSmartPointer<vtkFloatArray> normals = vtkSmartPointer<vtkFloatArray>::New();
    normals->SetNumberOfComponents(3);
    normals->SetName("Normals");

    for (unsigned int vertexIndex = 0; vertexIndex < mesh->mNumVertices; ++vertexIndex) {
      const aiVector3D& pos = mesh->mVertices[vertexIndex];
      points->InsertNextPoint(pos.x, pos.y, pos.z);

      if (mesh->HasNormals()) {
        const aiVector3D& n = mesh->mNormals[vertexIndex];
        normals->InsertNextTuple3(n.x, n.y, n.z);
      } else {
        normals->InsertNextTuple3(0.0, 0.0, 1.0);
      }
    }

    for (unsigned int faceIndex = 0; faceIndex < mesh->mNumFaces; ++faceIndex) {
      const aiFace& face = mesh->mFaces[faceIndex];
      if (face.mNumIndices == 3) {
        vtkIdType pts[3] = {
          static_cast<vtkIdType>(face.mIndices[0]),
          static_cast<vtkIdType>(face.mIndices[1]),
          static_cast<vtkIdType>(face.mIndices[2])
        };
        polys->InsertNextCell(3, pts);
      }
    }

    polyData->SetPoints(points);
    polyData->SetPolys(polys);
    if (polyData->GetNumberOfPoints() == normals->GetNumberOfTuples()) {
      polyData->GetPointData()->SetNormals(normals);
    }

    vtkAssImpMeshPart part;
    part.PolyData = polyData;
    part.Color = resolveMeshColor(mesh);
    parts.push_back(part);
  }

  return parts;
}

vtkSmartPointer<vtkPolyData> vtkAssImpConversion::vtkAssImpToPolyData(const std::string& filename, std::vector<double>& color)
{
  std::vector<vtkAssImpMeshPart> parts = vtkAssImpConversion::vtkAssImpToPolyDataParts(filename);
  if (parts.empty()) {
    color = {0.5, 0.5, 0.5};
    return nullptr;
  }

  vtkNew<vtkAppendPolyData> appendFilter;
  for (const auto& part : parts) {
    if (part.PolyData) {
      appendFilter->AddInputData(part.PolyData);
    }
  }
  appendFilter->Update();

  vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
  polyData->ShallowCopy(appendFilter->GetOutput());

  color = parts.front().Color;

  return polyData;
}
