osm2pgrouting \
	-f "$1" \
	-c "/usr/share/osm2pgrouting/mapconfig.xml" \
	-d gisdb \
	-U postgres \
	-W password
