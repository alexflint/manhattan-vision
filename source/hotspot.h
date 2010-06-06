#include <sstream>

#include "common_types.h"
#include "widget3d.h"

namespace indoor_context {
	// A hotspot specification
	struct HotSpot {
		shared_ptr<ostringstream> text;
		toon::Vector<3> worldPos;
		toon::Vector<2> screenPos;
		float radius;
	};

	// A widget that prints some info when one of its hotspots is clicked
	class HotSpots : public Widget3D {
	public:
		static const float kDefaultRadius = 8;
		vector<HotSpot> spots;
		void Draw();
		ostream& Add(toon::Vector<3> p, float rad = kDefaultRadius);
		bool HitTest(const toon::Vector<2>& mouse) const;
		void OnClick(int button, int x, int y);
	};
}
