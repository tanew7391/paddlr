import {React} from "react";
import "./Button.css";

const Button = ({ onClick, children, disabled }) => {
    return (
        <button
            onClick={onClick}
            className="custom-button"
            disabled={disabled}
        >
            {children}
        </button>
    );
}

export default Button;