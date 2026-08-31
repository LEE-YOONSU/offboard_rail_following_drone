# Reference basis

This scene is a standards-based Korean conventional railway reference segment. It is digital-twin-ready, but it is not an as-built twin of a named site until survey coordinates, alignment, terrain and asset inspection data are supplied.

## Applied dimensional rules

- Gauge: 1.435 m.
- Target visual class: conventional main line at or below 120 km/h, represented with a simplified 50N rail profile.
- PC sleeper length: 2.4 m.
- Long-welded ballasted-track sleeper spacing: 0.60 m.
- Contact-wire nominal height: 5.20 m above rail level.
- Simple-catenary system height for the 70–200 km/h class: 0.96 m.
- Dropper spacing: nominally 5 m.
- Mast span used in this scene: 48 m.
- Contact-wire support stagger: alternating ±0.20 m.
- The 50N visual profile now separates the oxidized foot/web/head sides from polished running and gauge-contact bands, with a small inward rail-seat inclination.
- Each PSC sleeper carries distinct steel seat plates, black resilient rail pads, insulating shoulders, bolts and rust-toned e-clip approximations.
- A deterministic low-poly angular aggregate layer sits above the textured trapezoidal ballast bed, avoiding the flat-surface appearance while keeping collision geometry simple.
- Main-line geometry: 252 m tangent followed by a 248 m circular curve of 400 m radius; total route length remains 500 m.
- Curved overhead contact line uses 24 m chord modules to keep lateral chord error small while retaining the nominal contact-wire height.
- A right-hand turnout at chainage 330 m feeds a curved, non-electrified 68 m siding with point blade, check-rail, switch-machine and buffer-stop visual detail.

## Primary references

1. Korea National Railway, **KR C-14060 Track Material Design, Rev.9 (2025-12-31)**  
   https://www.kr.or.kr/boardCnts/view.do?boardID=1000009&boardSeq=1121551
2. Korea National Railway, **KR E-03130 Composite Catenary Design, Rev.9 (2025-03-28)**  
   https://www.kr.or.kr/boardCnts/view.do?boardID=1000009&boardSeq=1120747
3. Korea National Railway, **KR E-03160 Contact Wire Height and Gradient, Rev.5 (2021-12-29)**  
   https://www.kr.or.kr/boardCnts/view.do?boardID=1000009&boardSeq=1116043
4. Railway Industry Information Center, **Overhead contact line overview and component terminology**  
   https://www.kric.go.kr/jsp/board/portal/sub05/knp/railCommonSenceDetail.jsp?p_id1=A010031131&p_id2=214
5. Korea National Railway, **Track construction overview and material categories**  
   https://www.kr.or.kr/sub/info.do?m=05040301

## Visual reference observations

- PC sleepers and gray angular ballast dominate modern conventional-line track appearance.
- Galvanised steel H-section masts stand outside the ballast shoulder on concrete foundations.
- Contact wire is nearly level; messenger wire visibly sags and connects through unequal droppers.
- Cable troughs, drainage, boundary fencing, equipment cabinets and uneven vegetation are necessary for a credible rail corridor silhouette.
- The supplied station photograph was used as the visual basis for a pale concrete side platform, yellow tactile strip, white / galvanized canopy frame, shallow metal roof, suspended signs, bench and glass windscreen. It is a type reference, not a reconstruction of the photographed station.
- The road intersection is implemented as a Korean-style level crossing with rubber deck panels, stop lines, warning heads, crossbucks and lowered red-white barriers.

## Synthetic-data basis

- The active world contains no fixed cameras. A co-registered RGB, depth, semantic and instance sensor file is retained only as a future drone-mounted template.
- Semantic class IDs are stable and recorded in `config/labels.yaml`; instance segmentation remains unique per rendered entity.
- The Gazebo Harmonic Sensors and Label systems are used rather than material-color approximations. The official Harmonic feature matrix lists depth, segmentation and bounding-box cameras, and the official segmentation camera example documents semantic / instance modes and colored / label-map outputs.
- Generated platform-concrete and crossing-asphalt albedo maps are deliberately seamless, shadow-free and without unique landmark stains to reduce baked-lighting and tiling artifacts in randomized captures.

### Gazebo sensor references

1. Gazebo Harmonic feature comparison: https://gazebosim.org/docs/harmonic/comparison/
2. Gazebo Sensors segmentation camera example: https://gazebosim.org/api/sensors/9/segmentationcamera_igngazebo.html
3. Gazebo Sensors bounding-box camera reference: https://gazebosim.org/api/sensors/9/boundingbox_camera.html

## Required inputs for an as-built twin

- Surveyed alignment or LandXML / IFC / GIS centreline
- Geodetic origin and heading
- LiDAR or photogrammetry point cloud
- Mast, signal, cabinet and structure inventory with chainage
- Actual rail profile, cant, gradient and curvature records
- Terrain DEM and orthophotos with redistribution rights
