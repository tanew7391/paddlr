import React, { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { MapContainer, TileLayer, useMapEvents, Marker, GeoJSON, useMap } from "react-leaflet";
import "leaflet/dist/leaflet.css";
import L from "leaflet";



import { getRoute } from "util/api";
import { marker_array_to_multipoint_geojson } from "util/util"
import { MINIMUM_WAYPOINTS, STARTING_LATITUDE, STARTING_LONGITUDE } from "util/constants";

L.Marker.prototype.options.icon = L.icon({
    iconUrl: "https://unpkg.com/leaflet@1.7.1/dist/images/marker-icon.png"
});


const MapRender = () => {
    const [waypoints, setWaypoints] = useState([]);
    const [route, setRoute] = useState(null);
    const [error, setError] = useState(null);
    const geoJsonLayer = useRef();

    const parentMap = useMap();

    useMapEvents({
        click(e) {
            const waypoint = (
                <Marker
                    position={e.latlng}
                >
                </Marker>
            );

            setWaypoints(oldWaypoints => {
                return [...oldWaypoints, waypoint];
            });
        }
    });
    
    const onClearMap = useCallback(() => {
        setWaypoints([]);
        setRoute(null);
        if (geoJsonLayer.current) {
            geoJsonLayer.current.clearLayers(); // Clear the existing layer
        }
    }, []);

    L.Control.Button = L.Control.extend({
        options: {
            position: 'topleft'
        },
        onAdd: (map) => {
            var container = L.DomUtil.create('div', 'leaflet-bar leaflet-control');
            var button = L.DomUtil.create('a', 'leaflet-control-button', container);
            button.innerHTML = '<img src="https://icons.getbootstrap.com/assets/icons/trash.svg" style="width:16px; padding-top: 7px;"/>';
            L.DomEvent.disableClickPropagation(button);
            L.DomEvent.on(button, 'click', function(e){
                onClearMap()
            });

            container.title = "Clear Map";

            return container;
        },
        onRemove: (map) => {
        },
    });

    let control = useMemo(() => new L.Control.Button(), []); 
    useEffect(() => {control.addTo(parentMap)}, [control, parentMap]);;

    useEffect(() => {
        console.log("Waypoints changed:", waypoints);
        setError(null);
        if (waypoints.length > MINIMUM_WAYPOINTS) {
            getRoute(
                marker_array_to_multipoint_geojson(waypoints)
            ).then((info) => {
                console.log("Setting route");
                setRoute(info.data);
            }).catch((err) => {
                setError(err.message || "An error occurred while fetching the route.");
            })
        }
    }, [waypoints]);



    useEffect(() => {
        if (geoJsonLayer.current) {
            geoJsonLayer.current.clearLayers(); // Clear the existing layer
            geoJsonLayer.current.addData(route); // Add the new data
        }
    }, [route]);

    return (
        <div>
            {error && 
                <div 
                    className="leaflet-center leaflet-control"
                    style={{
                        background: "white",
                        padding: "8px 16px",
                        borderRadius: "4px",
                        boxShadow: "0 2px 8px rgba(0,0,0,0.15)",
                        maxWidth: "80vw",
                        textAlign: "center",
                        margin: "16px auto",
                        fontSize: "25px",
                        position: "absolute",
                        left: 0,
                        right: 0,
                        top: "50%",
                        pointerEvents: "auto"
                    }}
                >
                    {error}
                </div>
            }
            <span>
                {waypoints.map((waypoint, index) => (
                    <React.Fragment key={index}>
                        {waypoint}
                    </React.Fragment>
                ))}
                {
                    route != null && <GeoJSON data={route} ref={geoJsonLayer} />
                }
            </span>
        </div>
    )
}

const SimpleMap = () => {
    const mapRef = useRef(null);
    const latitude = STARTING_LATITUDE;
    const longitude = STARTING_LONGITUDE;

    return (
        <>
            <MapContainer center={[latitude, longitude]} zoom={13} ref={mapRef} style={{ height: "100vh", width: "100vw" }}>
                <TileLayer
                    attribution='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
                    url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
                />
                <MapRender />
            </MapContainer>
        </>
    );
};

export default SimpleMap;