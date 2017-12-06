#ifndef OBJECTIVE_FUNCTION_AD
#define OBJECTIVE_FUNCTION_AD

#include "ObjectiveFunction.h"
#include <MathLib/AutoDiff.h>

class ObjectiveFunctionAD : public ObjectiveFunction{
public:
	ObjectiveFunctionAD();
	virtual ~ObjectiveFunctionAD();

	virtual double computeValue(const dVector& p);
	virtual void addGradientTo(dVector& grad, const dVector& p);
	virtual void addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p);

protected:
	template<class T>
	struct DOF {
		DOF(T* v, int i)
			: v(v), i(i){}
		T* v;
		int i;
	};

	template<class T>
	struct DOFColl {
		std::vector<DOF<T>> dofs;

		std::map<std::string, T> scalars;
		std::map<std::string, Vector3T<T>> vector3s;
		std::map<std::string, VectorXT<T>> vectorXs;

		void addScalar(const dVector &p, int index, const std::string &name){
			scalars[name] = p[index];
			dofs.push_back(DOF<T>(&scalars[name], index));
		}

		void addVector3(const dVector &p, int index, const std::string &name){
			vector3s[name] = Vector3T<T>(p[index+0], p[index+1], p[index+2]);
			dofs.push_back(DOF<T>(&vector3s[name](0), index+0));
			dofs.push_back(DOF<T>(&vector3s[name](1), index+1));
			dofs.push_back(DOF<T>(&vector3s[name](2), index+2));
		}

		void addVectorX(const dVector &p, int index, int size, const std::string &name){
			vectorXs[name] = VectorXT<T>(size);
			for (int i = 0; i < size; ++i) {
				vectorXs[name][i] = p[index+i];
				dofs.push_back(DOF<T>(&vectorXs[name](i), index+i));
			}
		}
	};

protected:
	typedef AutoDiffT<double, double> ScalarDiff;
	typedef AutoDiffT<ScalarDiff, ScalarDiff> ScalarDiffDiff;

	virtual double computeEnergy(const DOFColl<double> &dofColl) = 0;
	virtual ScalarDiff computeEnergy(const DOFColl<ScalarDiff> &dofColl) = 0;
	virtual ScalarDiffDiff computeEnergy(const DOFColl<ScalarDiffDiff> &dofColl) = 0;

	virtual DOFColl<double> collectDOFs(const dVector &p) = 0;
	virtual DOFColl<ScalarDiff> collectDOFsD(const dVector &p) = 0;
	virtual DOFColl<ScalarDiffDiff> collectDOFsDD(const dVector &p) = 0;


};

#endif // OBJECTIVE_FUNCTION_AD
