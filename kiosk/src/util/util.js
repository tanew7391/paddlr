export const marker_array_to_multipoint_geojson = (marker_array) => {
    const coordinates = marker_array.map(marker => {
        const { lat, lng } = marker.props.position;
        return [lng, lat];  // GeoJSON format uses [longitude, latitude]
    });

    const geoJsonMultiPoint = {
            type: "MultiPoint",
            coordinates: coordinates
    };

    return geoJsonMultiPoint
}