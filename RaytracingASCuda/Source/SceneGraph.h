#pragma once
#include <vector>


class SceneGraph final
{
public:

	static SceneGraph* GetInstance();
	static void CleanUp();

	

	~SceneGraph();
	SceneGraph(const SceneGraph&) = delete;
	SceneGraph(SceneGraph&&) noexcept = delete;
	SceneGraph& operator=(const SceneGraph&) = delete;
	SceneGraph& operator=(SceneGraph&&) noexcept = delete;
private :
	SceneGraph() = default;
	static SceneGraph* m_pInstance;

};

