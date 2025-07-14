import React, { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { MapContainer, TileLayer, useMapEvents, Marker, GeoJSON, useMap } from "react-leaflet";
import { OpenStreetMapProvider, GeoSearchControl } from 'leaflet-geosearch';
import "leaflet/dist/leaflet.css";
import L from "leaflet";

import { getRoute } from "util/api";
import { marker_array_to_multipoint_geojson } from "util/util"
import { MINIMUM_WAYPOINTS, STARTING_LATITUDE, STARTING_LONGITUDE } from "util/constants";
import LoadingIndicator from "components/LoadingIndicator/LoadingIndicator";

L.Marker.prototype.options.icon = L.icon({
    iconUrl: "https://unpkg.com/leaflet@1.7.1/dist/images/marker-icon.png"
});


const MapRender = () => {
    const [waypoints, setWaypoints] = useState([]);
    const [route, setRoute] = useState(null);
    const [error, setError] = useState(null);
    const [total_distance, setTotalDistance] = useState("0.00 km");
    const [loading, setLoading] = useState(false);

    const mapRef = useRef();

    const parentMap = useMap();

    const search = new GeoSearchControl({
        provider: new OpenStreetMapProvider(),
        style: 'bar',
    });

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

    useEffect(() => {
        parentMap.addControl(search);

    // eslint-disable-next-line react-hooks/exhaustive-deps
    }, []);

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
        setTotalDistance("0.00 km");
        setError(null);
        if (mapRef.current) {
            mapRef.current.clearLayers(); // Clear the existing layer
        }
    }, []);

    let control = useMemo(() => new L.Control.Button(), []); 
    useEffect(() => {control.addTo(parentMap)}, [control, parentMap]);;

    useEffect(() => {
        console.log("Waypoints changed:", waypoints);
        setError(null);
        if (waypoints.length > MINIMUM_WAYPOINTS) {
            setLoading(true);
            getRoute(
                marker_array_to_multipoint_geojson(waypoints)
            ).then((info) => {
                console.log("Setting route");
                setRoute(info.data);
            }).catch((err) => {
                setError(err.message || "An error occurred while fetching the route.");
            }).finally(() => {
                setLoading(false);
            });
        }
    }, [waypoints]);



    useEffect(() => {
        if (mapRef.current) {
            mapRef.current.clearLayers(); // Clear the existing layer
            let myLayer = L.geoJSON().addTo(mapRef.current);
            myLayer.addData(route)

            let total_distance = 0;


            route.geometries.forEach(ls => {
                if (ls.type !== "LineString") return;
                // Calculate the total length of the LineString
                const latlngs = ls.coordinates.map(([lng, lat]) => L.latLng(lat, lng));
                for (let i = 1; i < latlngs.length; i++) {
                    total_distance += parentMap.distance(latlngs[i - 1], latlngs[i]);
                }
            });

            setTotalDistance((total_distance / 1000).toFixed(2) + " km");

        }
    }, [parentMap, route]);

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
            {loading && 
                <div 
                    className="leaflet-control leaflet-center "
                    style={{
                        position: "absolute",
                        left: 0,
                        right: 0,
                    }}
                >
                    <LoadingIndicator />
                </div>
            }
            <div className="leaflet-bottom leaflet-control" style={{ textAlign: "center", padding: "8px 16px", background: "white", borderRadius: "4px", boxShadow: "0 2px 8px rgba(0,0,0,0.15)", maxWidth: "80vw", margin: "16px auto" }}>
                <h2>{total_distance}</h2>
            </div>
            <span>
                {waypoints.map((waypoint, index) => (
                    <React.Fragment key={index}>
                        {waypoint}
                    </React.Fragment>
                ))}
                {
                    route != null && <GeoJSON data={route} ref={mapRef} />
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