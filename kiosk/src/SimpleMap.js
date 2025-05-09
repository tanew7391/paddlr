import React, { useEffect, useRef, useState } from "react";
import { MapContainer, TileLayer, useMapEvents, Marker, GeoJSON } from "react-leaflet";
import "leaflet/dist/leaflet.css";
import L from "leaflet";



import { getRoute } from "util/api";
import { marker_array_to_multipoint_geojson } from "util/util"
import { MINIMUM_WAYPOINTS, STARTING_LATITUDE, STARTING_LONGITUDE } from "util/constants";

L.Marker.prototype.options.icon = L.icon({
    iconUrl: "https://unpkg.com/leaflet@1.7.1/dist/images/marker-icon.png"
});

const Tester = () => {
    const [waypoints, setWaypoints] = useState([]);
    const [route, setRoute] = useState(null);
    const geoJsonLayer = useRef();

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

    useEffect(() => {
        if (waypoints.length > MINIMUM_WAYPOINTS) {
            getRoute(
                marker_array_to_multipoint_geojson(waypoints)
            ).then((info) => {
                console.log("Setting route");
                setRoute(info.data);
            }).catch((err) => {
                console.error(err);
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
    )
}

const SimpleMap = () => {
    const mapRef = useRef(null);
    const latitude = STARTING_LATITUDE;
    const longitude = STARTING_LONGITUDE;

    return (
        // Make sure you set the height and width of the map container otherwise the map won't show
        <MapContainer center={[latitude, longitude]} zoom={13} ref={mapRef} style={{ height: "100vh", width: "100vw" }}>
            <TileLayer
                attribution='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
                url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
            />
            {/* Additional map layers or components can be added here */}
            <Tester />
        </MapContainer>
    );
};

export default SimpleMap;