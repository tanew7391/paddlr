import React from "react";

import loadingGif from "assets/loading.gif";
import "./LoadingIndicator.css";

const LoadingIndicator = () => (
  <div
    className="loadingIndicatorContainer"
  >
    <div className="imageContainer">
      <img
        src={loadingGif}
        alt="Loading..."
        className="loadingIndicatorImage"
      />
      <h2
        className="loadingText"
      >
        Loading...
      </h2>
    </div>
  </div>
);

export default LoadingIndicator;