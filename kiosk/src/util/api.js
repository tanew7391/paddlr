import axios from "axios";

export const getRoute = async (waypoints) => {
    const response = await axios.post("/api/", {
        ...waypoints
    });
    return response;
}

