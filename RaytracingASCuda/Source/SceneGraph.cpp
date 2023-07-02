#include "SceneGraph.h"
SceneGraph* SceneGraph::m_pInstance{};

SceneGraph* SceneGraph::GetInstance()
{
	if (m_pInstance == nullptr)
	{
		m_pInstance = new SceneGraph();

	}
	return m_pInstance;
}

void SceneGraph::CleanUp()
{
	delete m_pInstance;
	m_pInstance = nullptr;
}



SceneGraph::~SceneGraph()
{

}
