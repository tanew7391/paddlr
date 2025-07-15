import axios from "axios";

export const getRoute = async (stop_flag, geometry) => {
    const response = await axios.post("/api/", {
        stop_flag: stop_flag,
        geometry: {...geometry}
    });
    return response;
}

