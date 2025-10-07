import { React } from "react";
import "./Dropdown.css";

const Dropdown = ({setFlag, selectedFlag, options}) => {
    
    
    return (
            <select
                value={selectedFlag}
                onChange={(e) => setFlag(e.target.value)}
                className="dropdown"
            >
                {options.map((option, index) => (
                    <option key={index} value={option.value}>
                        {option.label}
                    </option>
                ))}
            </select>
    );
}

export default Dropdown;