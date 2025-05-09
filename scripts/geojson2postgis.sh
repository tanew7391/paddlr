#need to run as user postgres
ogr2ogr -f "PostgreSQL" PG:"dbname=gisdb user=postgres" "algonquin_long.geojson" -nln points -append -t_srs EPSG:3857