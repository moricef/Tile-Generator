#!/usr/bin/env python3
"""
PBF to NAV Tile Converter

Converts OpenStreetMap .pbf files to NAV binary format (.nav) with tile structure.
NAV format optimized for ESP32:
- 22-byte tile header (Magic, Count, Bbox)
- 11-byte feature header (Type, Color, Zoom/Priority, Width, BBox, Count)
- int16 relative coordinates (0-4096 range with safety margin)
- ~50% size reduction vs previous version

Width field encoding (1 byte):
- Bit 7 (0x80): Casing flag for two-pass rendering (motorway/trunk/primary)
- Bits 0-6 (0x7F): Actual width in pixels (0-127)

Usage:
    python tile_generator.py input.pbf output_dir features.json [--zoom 6-17]
"""

import os
import sys
import json
import argparse
import logging
import math
import struct
from typing import Dict, List, Tuple, Set, Any, Optional
from collections import defaultdict
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
import multiprocessing

try:
    import osmium
    from osmium import osm
    import osmium.geom
    import osmium.area
except ImportError:
    print("Error: osmium not found. Install with: pip install osmium")
    sys.exit(1)

try:
    from shapely.geometry import Polygon
    import shapely.wkb
    SHAPELY_AVAILABLE = True
except ImportError:
    SHAPELY_AVAILABLE = False
    print("Warning: shapely not found. Multipolygon support disabled.")

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# NAV format constants
NAV_MAGIC = b'NAV1'
COORD_SCALE = 10000000  # 1e7 for ~1cm precision
LAND_BG_COLOR = '#f2efe9'  # OSM Carto default land background

# Geometry types
GEOM_POINT = 1
GEOM_LINESTRING = 2
GEOM_POLYGON = 3
GEOM_TEXT = 4

# Perceptual filtering: minimum visible area in pixels squared
K_VISIBILITY = 2.0
# Anti-pitting: holes must be N times more visible than objects to be kept
K_HOLE_FACTOR = 10.0

# Point features to extract from nodes (rendered as symbols)
# shape: 'triangle' for peaks, 'circle' for places
POINT_FEATURES = {
    'natural=peak': 'triangle',
    'natural=volcano': 'triangle',
}

# Place features to extract as text labels
# Maps to (base_font_size, base_zoom, population_zoom_rules)
# population_zoom_rules: list of (min_pop, zoom) sorted descending
TEXT_FEATURES = {
    'place=city': {
        'font_size': 2,
        'zoom_rules': [(1000000, 4), (500000, 5), (100000, 6), (0, 8)],
    },
    'place=town': {
        'font_size': 1,
        'zoom_rules': [(50000, 8), (15000, 9), (5000, 10), (0, 11)],
    },
    'place=village': {
        'font_size': 0,
        'zoom_rules': [(2000, 11), (500, 12), (0, 13)],
    },
    'place=suburb': {
        'font_size': 0,
        'zoom_rules': [(0, 12)],
    },
    'place=hamlet': {
        'font_size': 0,
        'zoom_rules': [(0, 14)],
    },
}

# Tags that support width (LineStrings only)
WIDTH_TAGS = {'highway', 'railway', 'waterway'}

# Fixed width in pixels per feature type and zoom level (OSM Carto style)
# Format: type_value -> {zoom: pixels}
LINE_WIDTH_PER_ZOOM = {
    # z6-z11: original OSM Carto widths (sausage artifact not visible at low zoom)
    # z12+: halved (min 1) to avoid fillCircle sausage artifact on T-Deck
    'motorway':      {6: 1,  7: 1,  8: 2,  9: 2,  10: 2,  11: 3,  12: 3,  13: 3,  14: 3,  15: 3,  16: 5,  17: 9,  18: 11, 19: 14},
    'motorway_link': {                            10: 1,  11: 2,  12: 1,  13: 2,  14: 2,  15: 2,  16: 4,  17: 6,  18: 7,  19: 8},
    'trunk':         {6: 1,  7: 1,  8: 2,  9: 2,  10: 2,  11: 2,  12: 2,  13: 3,  14: 3,  15: 3,  16: 5,  17: 9,  18: 11, 19: 14},
    'trunk_link':    {                            10: 1,  11: 2,  12: 1,  13: 2,  14: 2,  15: 2,  16: 4,  17: 6,  18: 7,  19: 8},
    'primary':       {              8: 1,  9: 1,  10: 2,  11: 2,  12: 2,  13: 3,  14: 3,  15: 3,  16: 5,  17: 9,  18: 11, 19: 14},
    'primary_link':  {                            10: 1,  11: 1,  12: 1,  13: 2,  14: 2,  15: 2,  16: 4,  17: 6,  18: 7,  19: 8},
    'secondary':     {                            10: 1,  11: 1,  12: 2,  13: 3,  14: 3,  15: 2,  16: 5,  17: 9,  18: 11, 19: 14},
    'secondary_link':{                            10: 1,  11: 1,  12: 1,  13: 2,  14: 2,  15: 2,  16: 4,  17: 6,  18: 7,  19: 8},
    'tertiary':      {                            10: 1,  11: 1,  12: 1,  13: 2,  14: 3,  15: 2,  16: 5,  17: 9,  18: 11, 19: 14},
    'tertiary_link': {                                            12: 1,  13: 1,  14: 2,  15: 2,  16: 4,  17: 6,  18: 7,  19: 8},
    'residential':   {                                                    13: 1,  14: 2,  15: 2,  16: 3,  17: 6,  18: 7,  19: 9},
    'pedestrian':    {                                                    13: 1,  14: 2,  15: 2,  16: 3,  17: 6,  18: 7,  19: 9},
    'living_street': {                                                    13: 1,  14: 2,  15: 2,  16: 3,  17: 6,  18: 7,  19: 9},
    'unclassified':  {                                            12: 1,  13: 2,  14: 2,  15: 2,  16: 3,  17: 6,  18: 7,  19: 9},
    'service':       {                                                    13: 1,  14: 1,  15: 1,  16: 2,  17: 3,  18: 4,  19: 5},
    'track':         {                                                    15: 1,  16: 1,  17: 2,  18: 2,  19: 3},
    'footway':       {                                                    13: 1,  14: 1,  15: 1,  16: 1,  17: 1,  18: 1,  19: 1},
    'cycleway':      {                                                    13: 1,  14: 1,  15: 1,  16: 1,  17: 1,  18: 1,  19: 1},
    'path':          {                                                    13: 1,  14: 1,  15: 1,  16: 1,  17: 1,  18: 1,  19: 1},
    'bridleway':     {                                                    13: 1,  14: 1,  15: 1,  16: 1},
    # Railway - based on OpenStreetMap Carto standard (halved from original, min 1)
    'rail':          {                     9: 1,  10: 1,  11: 1,  12: 1,  13: 2,  14: 2,  15: 2,  16: 2,  17: 2,  18: 3,  19: 4},
    'subway':        {                                            12: 1,  13: 1,  14: 1,  15: 2,  16: 2},
    'tram':          {                                                                    15: 1,  16: 2},
    'narrow_gauge':  {                                                    13: 1,  14: 1,  15: 2,  16: 2},
    'funicular':     {                                                    13: 1,  14: 1,  15: 2,  16: 2},
    # Aeroway - typical runway ~45m, taxiway ~23m, helipad ~15m (halved from original, min 1)
    'runway':        {                                              12: 2,  13: 3,  14: 4,  15: 6,  16: 8,  17: 11, 18: 14},
    'taxiway':       {                                              12: 1,  13: 1,  14: 2,  15: 2,  16: 3,  17: 5,  18: 7},
    'helipad':       {                                              12: 1,  13: 2,  14: 2,  15: 2,  16: 3,  17: 4,  18: 5},
}

# Override color per zoom level (RGB565) - only for features that change color by zoom
# Format: type_value -> {zoom: '#hexcolor'}
# If a zoom is not listed, the default JSON color is used
LINE_COLOR_PER_ZOOM = {
    'residential':   {13: '#cccccc'},
    'unclassified':  {12: '#cccccc'},
    'living_street': {12: '#cccccc'},
    'track':         {15: '#ffffff', 16: '#ffffff'},
    'service':       {16: '#cccccc'},
    'secondary':     {10: '#bababa', 11: '#bababa'},
}

# Cache for zoom level parameters
_ZOOM_PARAMS_CACHE = {}


def _get_zoom_params(zoom: int) -> Dict:
    """Get cached zoom parameters or compute them."""
    if zoom not in _ZOOM_PARAMS_CACHE:
        n = 2.0 ** zoom
        _ZOOM_PARAMS_CACHE[zoom] = {
            'n': n,
            'lon_scale': n / 360.0,
            'lon_offset': 180.0
        }
    return _ZOOM_PARAMS_CACHE[zoom]


def meters_to_pixels(width_meters: float, zoom: int, lat: float = 45.0) -> int:
    """Convert width in meters to pixels at given zoom level.

    Uses approximation for given latitude (default 45° for Europe).
    Formula: meters_per_pixel ≈ 156543 * cos(lat) / 2^zoom
    """
    meters_per_pixel = 156543.0 * math.cos(math.radians(lat)) / (2 ** zoom)
    pixels = int(width_meters / meters_per_pixel + 0.5)
    return max(1, min(15, pixels))  # Clamp to 1-15


# Layer rendering priority (lower = rendered first = behind)
LAYER_PRIORITY = {
    'aeroways': 5,
    'landuse': 10,
    'terrain': 20,
    'water': 30,
    'islands': 35,
    'amenities': 40,
    'railways': 45,
    'roads': 55,
    'infrastructure': 65,
    'buildings': 75,
    'leisure': 80,      # Sports fields above water (visible on islands)
    'surface': 86,      # Ground cover (grass, sand) visible inside leisure polygons (must be > max leisure combined=85)
    'parking': 88,      # Parking lots above leisure zones and surface cover
    'boundaries': 85,
    'places': 90
}

# Layer definitions based on feature types
LAYER_MAPPING = {
    'water': [
        'natural=water', 'natural=coastline', 'natural=bay',
        'waterway=riverbank', 'waterway=dock', 'waterway=boatyard',
        'waterway=river', 'waterway=stream', 'waterway=canal',
        'waterway=ditch', 'waterway=drain',
        'natural=spring', 'natural=wetland',
        'water=river', 'water=canal', 'water=reservoir', 'water=pond', 'water=lake', 'water=basin',
        'landuse=reservoir'
    ],
    'islands': [
        'place=island', 'place=islet'
    ],
    'aeroways': [
        'aeroway=aerodrome'
    ],
    'landuse': [
        'natural=beach', 'natural=sand', 'natural=wood',
        'landuse=forest', 'natural=forest', 'natural=scrub',
        'natural=heath',
        'natural=bare_rock', 'natural=rock', 'natural=scree', 'natural=stone',
        'natural=fell', 'natural=moor', 'natural=shrubbery', 'landuse=quarry',
        'landuse=orchard', 'landuse=vineyard',
        'landuse=farmland', 'landuse=farmyard',
        'landuse=residential',
        'landuse=commercial', 'landuse=retail', 'landuse=industrial',
        'landuse=construction', 'landuse=cemetery', 'landuse=allotments',
        'leisure=common', 'landuse=village_green',
        'landuse=quarry', 'landuse=military', 'landuse=landfill', 'landuse=brownfield',
        'landuse=basin', 'landuse=railway', 'landuse=education',
        'landuse=garages', 'landuse=flowerbed'
    ],
    'surface': [
        'natural=grassland', 'landuse=grass', 'landuse=meadow',
    ],
    'roads': [
        'highway=motorway', 'highway=motorway_link',
        'highway=trunk', 'highway=trunk_link',
        'highway=primary', 'highway=primary_link',
        'highway=secondary', 'highway=secondary_link',
        'highway=tertiary', 'highway=tertiary_link',
        'highway=residential', 'highway=living_street',
        'highway=unclassified', 'highway=service',
        'highway=pedestrian', 'highway=track',
        'highway=path', 'highway=footway',
        'highway=cycleway', 'highway=steps',
        'highway=crossing', 'highway=bus_stop',
        'highway=construction', 'highway=platform'
    ],
    'railways': [
        'railway=rail', 'railway=subway', 'railway=tram',
        'railway=abandoned', 'railway=disused', 'railway=funicular',
        'railway=narrow_gauge', 'railway=platform'
    ],
    'buildings': [
        'building', 'man_made=tower'
    ],
    'amenities': [
        'amenity=hospital',
        'amenity=school', 'amenity=university',
        'amenity=place_of_worship',
        'amenity=grave_yard', 'amenity=marketplace',
    ],
    'parking': [
        'amenity=parking',
        'amenity=parking_space',
    ],
    'infrastructure': [
        'bridge=yes', 'man_made=bridge',
        'aeroway=runway', 'aeroway=taxiway', 'aeroway=apron',
        'aeroway=hangar', 'aeroway=helipad',
        'aeroway=parking_position',
        'tunnel=yes', 'tunnel=culvert',
        'man_made=embankment', 'man_made=pier',
        'waterway=dam', 'waterway=weir'
    ],
    'terrain': [
        'natural=peak', 'natural=ridge',
        'natural=volcano', 'natural=cliff',
        'natural=tree_row', 'natural=tree',
        'natural=arete', 'natural=earth_bank',
        'natural=shingle', 'natural=glacier'
    ],
    'boundaries': [
        'boundary=administrative'
    ],
    'leisure': [
        'leisure=pitch', 'leisure=stadium', 'leisure=sports_centre',
        'leisure=sports_hall', 'leisure=track', 'leisure=swimming_pool',
        'leisure=golf_course', 'leisure=playground',
        'landuse=park', 'leisure=park', 'leisure=nature_reserve', 'leisure=garden',
        'leisure=recreation_ground', 'landuse=recreation_ground'
    ],
    'places': [
        'place=city', 'place=state', 'place=town',
        'place=village', 'place=hamlet', 'place=square'
    ]
}


def lon_to_tile_x(lon: float, zoom: int) -> int:
    """Convert longitude to tile X coordinate."""
    params = _get_zoom_params(zoom)
    return int((lon + params['lon_offset']) * params['lon_scale'])


def lat_to_tile_y(lat: float, zoom: int) -> int:
    """Convert latitude to tile Y coordinate."""
    params = _get_zoom_params(zoom)
    lat_rad = math.radians(lat)
    return int((1.0 - math.asinh(math.tan(lat_rad)) / math.pi) / 2.0 * params['n'])


def get_feature_tiles(coords: List[Tuple[float, float]], zoom: int, is_polygon: bool = False) -> Set[Tuple[int, int]]:
    """Get all tiles that a feature intersects at given zoom level."""
    tiles = set()

    if is_polygon and len(coords) >= 3:
        # Calculate min/max without creating intermediate lists
        min_lon = max_lon = coords[0][0]
        min_lat = max_lat = coords[0][1]
        
        for lon, lat in coords[1:]:
            if lon < min_lon: min_lon = lon
            elif lon > max_lon: max_lon = lon
            if lat < min_lat: min_lat = lat
            elif lat > max_lat: max_lat = lat

        min_x = lon_to_tile_x(min_lon, zoom)
        max_x = lon_to_tile_x(max_lon, zoom)
        min_y = lat_to_tile_y(max_lat, zoom)
        max_y = lat_to_tile_y(min_lat, zoom)

        for x in range(min_x, max_x + 1):
            for y in range(min_y, max_y + 1):
                tiles.add((x, y))
    else:
        for lon, lat in coords:
            x = lon_to_tile_x(lon, zoom)
            y = lat_to_tile_y(lat, zoom)
            tiles.add((x, y))

    return tiles


def get_layer_for_tags(tags: Dict[str, str]) -> Optional[str]:
    """Determine which layer a feature belongs to based on its tags."""
    
    # Explicit rule for all water-related features
    if (tags.get('natural') == 'water' or
        tags.get('natural') == 'bay' or
        'waterway' in tags or
        'water' in tags or
        tags.get('landuse') == 'reservoir'):
        return 'water'
    
    # Explicit rule for all highways and railways (BEFORE boundary filter,
    # because some roads run along administrative boundaries and carry both tags)
    if 'highway' in tags:
        return 'roads'
    if 'railway' in tags:
        return 'roads'

    # Do not create polygons for abstract features like boundaries or place names
    if 'place' in tags or 'boundary' in tags or 'admin_level' in tags:
        return None

    # Explicit rule for buildings to ensure they are always on top of scenery
    if 'building' in tags or tags.get('aeroway') == 'hangar':
        return 'buildings'

    for layer_name, feature_keys in LAYER_MAPPING.items():
        for feature_key in feature_keys:
            if '=' in feature_key:
                key, value = feature_key.split('=', 1)
                if key in tags and tags[key] == value:
                    return layer_name
            else:
                if feature_key in tags:
                    return layer_name
    return None


def get_config_value_for_tags(
    tags: Dict[str, str], 
    config: Dict, 
    attribute: str, 
    default: Any
) -> Any:
    """
    Generic helper to get configuration values for feature tags, with key priority.
    """
    preferred_keys = ['building', 'natural', 'waterway', 'highway', 'railway', 'water']

    # 1. Prioritize preferred keys
    for key in preferred_keys:
        if key in tags:
            value = tags[key]
            # Try exact match first (key=value)
            feature_key = f"{key}={value}"
            if feature_key in config and isinstance(config[feature_key], dict):
                return config[feature_key].get(attribute, default)
            
            # Then try key-only match
            if key in config and isinstance(config[key], dict):
                return config[key].get(attribute, default)

    # 2. Check remaining tags
    for key, value in tags.items():
        if key in preferred_keys:
            continue
        
        feature_key = f"{key}={value}"
        if feature_key in config and isinstance(config[feature_key], dict):
            return config[feature_key].get(attribute, default)
        
        if key in config and isinstance(config[key], dict):
            return config[key].get(attribute, default)
    
    return default


def get_zoom_for_tags(tags: Dict[str, str], config: Dict) -> int:
    """Get minimum zoom level for feature based on config."""
    return get_config_value_for_tags(tags, config, 'zoom', 6)


def get_color_for_tags(tags: Dict[str, str], config: Dict) -> str:
    """Get color for feature based on config."""
    return get_config_value_for_tags(tags, config, 'color', '#FFFFFF')


def get_priority_for_tags(tags: Dict[str, str], config: Dict) -> int:
    """Get rendering priority for feature based on config."""
    return get_config_value_for_tags(tags, config, 'priority', 50)


def hex_to_rgb565(hex_color: str) -> int:
    """Convert hex color to RGB565 format."""
    try:
        if not hex_color or not hex_color.startswith("#"):
            return 0xFFFF
        r = int(hex_color[1:3], 16)
        g = int(hex_color[3:5], 16)
        b = int(hex_color[5:7], 16)
        return ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3)
    except (ValueError, IndexError):
        logger.warning(f"Invalid hex color format: {hex_color}, using default")
        return 0xFFFF


def lighten_rgb565(color: int, factor: float = 0.4) -> int:
    """Lighten RGB565 color."""
    r = ((color >> 11) & 0x1F) * 255 // 31
    g = ((color >> 5) & 0x3F) * 255 // 63
    b = (color & 0x1F) * 255 // 31
    r = min(255, int(r + (255 - r) * factor))
    g = min(255, int(g + (255 - g) * factor))
    b = min(255, int(b + (255 - b) * factor))
    return ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3)


def darken_rgb565(color: int, factor: float = 0.4) -> int:
    """Darken RGB565 color."""
    r = ((color >> 11) & 0x1F) * 255 // 31
    g = ((color >> 5) & 0x3F) * 255 // 63
    b = (color & 0x1F) * 255 // 31
    r = max(0, int(r * (1 - factor)))
    g = max(0, int(g * (1 - factor)))
    b = max(0, int(b * (1 - factor)))
    return ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3)


def pack_zoom_priority(min_zoom: int, priority_nibble: int) -> int:
    """Pack min_zoom and priority into a single byte."""
    return (min(min_zoom, 15) << 4) | (min(priority_nibble, 15) & 0x0F)


def get_simplify_tolerance(zoom: int) -> float:
    """Calculate simplification tolerance based on zoom level."""
    tile_width_degrees = 360.0 / (2.0 ** zoom)
    pixel_size_degrees = tile_width_degrees / 256.0
    return pixel_size_degrees * 0.25  # Reduced from 0.5 to preserve roundabouts and curves


class BoundaryScanner(osmium.SimpleHandler):
    """First pass: collect way IDs that are members of boundary relations."""

    def __init__(self, config: Dict, max_zoom: int):
        super().__init__()
        self.config = config
        self.max_zoom = max_zoom
        self.boundary_ways: Dict[int, List[Dict]] = {}  # way_id -> list of boundary configs

    def relation(self, r):
        tags = {tag.k: tag.v for tag in r.tags}
        if tags.get('type') != 'boundary' or tags.get('boundary') != 'administrative':
            return

        admin_level = tags.get('admin_level', '')
        feature_key = f"boundary=administrative;admin_level={admin_level}"
        if feature_key not in self.config or not isinstance(self.config[feature_key], dict):
            return

        cfg = self.config[feature_key]
        min_zoom = cfg.get('zoom', 6)
        if min_zoom > self.max_zoom:
            return

        color_rgb565 = hex_to_rgb565(cfg.get('color', '#000000'))
        priority = cfg.get('priority', 85)
        layer_base_priority = LAYER_PRIORITY.get('boundaries', 85)
        combined_priority = layer_base_priority + (priority % 10)

        width_pixels = cfg.get('width', 1)

        way_info = {
            'color_rgb565': color_rgb565,
            'zoom_priority': pack_zoom_priority(min_zoom, combined_priority),
            'width_pixels': width_pixels,
        }

        for member in r.members:
            if member.type == 'w':
                if member.ref not in self.boundary_ways:
                    self.boundary_ways[member.ref] = []
                self.boundary_ways[member.ref].append(way_info)


class OSMHandler(osmium.SimpleHandler):
    """Handler for processing OSM data from PBF files."""

    def __init__(self, config: Dict, zoom_range: Tuple[int, int]):
        super().__init__()
        self.config = config
        self.min_zoom, self.max_zoom = zoom_range
        self.features: List[Dict] = []
        self.boundary_ways: Dict[int, Dict] = {}  # Set by caller after BoundaryScanner
        self.stats = {
            'nodes_processed': 0,
            'text_labels': 0,
            'ways_processed': 0,
            'areas_processed': 0,
            'boundary_ways_extracted': 0,
            'features_extracted': 0,
            'features_filtered': 0,
            'area_no_config': 0,
            'area_no_layer': 0,
            'area_zoom_filtered': 0,
            'area_exception': 0,
            'area_boundary': 0
        }
        self.start_time = time.time()
        self.last_progress_time = time.time()
        self.progress_interval = 5
        self.interesting_tags = self._build_interesting_tags()
        self.processed_way_ids: Set[int] = set()
        self.wkbfab = osmium.geom.WKBFactory()
        self.road_label_counters: Dict[str, int] = defaultdict(int)  # ref -> segment count

    def _build_interesting_tags(self) -> Set[str]:
        """Build set of tag keys we're interested in."""
        tags = set()
        for key in self.config:
            if isinstance(self.config[key], dict):
                if '=' in key:
                    tag_key = key.split('=')[0]
                    tags.add(tag_key)
                else:
                    tags.add(key)
        return tags

    def _log_progress(self):
        """Log progress periodically."""
        current_time = time.time()
        if current_time - self.last_progress_time >= self.progress_interval:
            self.last_progress_time = current_time
            ways = self.stats['ways_processed']
            extracted = self.stats['features_extracted']
            elapsed = current_time - self.start_time
            mins, secs = divmod(int(elapsed), 60)
            print(f"\r  Progress: {ways:,} ways, {extracted:,} features [{mins}m {secs:02d}s]", end='', flush=True)

    def _has_interesting_tags(self, tags) -> bool:
        """Check if tags contain any interesting keys."""
        for tag in tags:
            if tag.k in self.interesting_tags:
                return True
        return False

    def _tags_to_dict(self, tags) -> Dict[str, str]:
        """Convert osmium tags to dictionary."""
        return {tag.k: tag.v for tag in tags}

    def _is_feature_in_config(self, tags: Dict[str, str]) -> bool:
        """Check if feature matches any entry in config."""
        for key, value in tags.items():
            feature_key = f"{key}={value}"
            if feature_key in self.config and isinstance(self.config[feature_key], dict):
                return True
            if key in self.config and isinstance(self.config[key], dict):
                return True
        return False

    def node(self, n):
        """Process node - extract point features (peaks) and text labels (places)."""
        if not n.location.valid():
            return

        tags = self._tags_to_dict(n.tags)

        for key, value in tags.items():
            feature_key = f"{key}={value}"

            # Point symbols (peaks, volcanoes)
            if feature_key in POINT_FEATURES and feature_key in self.config:
                cfg = self.config[feature_key]
                min_zoom = cfg.get('zoom', 12)
                if min_zoom > self.max_zoom:
                    return

                color_rgb565 = hex_to_rgb565(cfg.get('color', '#000000'))
                priority = cfg.get('priority', 30)
                layer = get_layer_for_tags({key: value})
                layer_base_priority = LAYER_PRIORITY.get(layer or 'terrain', 20)
                combined_priority = layer_base_priority + (priority % 10)

                self.features.append({
                    'geom_type': GEOM_POINT,
                    'coords': [(n.location.lon, n.location.lat)],
                    'color_rgb565': color_rgb565,
                    'zoom_priority': pack_zoom_priority(min_zoom, combined_priority),
                    'width_meters': 0.0,
                    'shape': POINT_FEATURES[feature_key],
                })
                self.stats['nodes_processed'] += 1
                return

            # Text labels (places)
            if feature_key in TEXT_FEATURES and feature_key in self.config:
                name = tags.get('name', '')
                if not name:
                    self.stats['features_filtered'] += 1
                    return

                text_cfg = TEXT_FEATURES[feature_key]
                cfg = self.config[feature_key]

                # Determine zoom from population
                population = 0
                pop_str = tags.get('population', '0')
                try:
                    population = int(pop_str.replace(',', '').replace(' ', ''))
                except (ValueError, TypeError):
                    pass

                min_zoom = text_cfg['zoom_rules'][-1][1]  # default: last rule
                for min_pop, z in text_cfg['zoom_rules']:
                    if population >= min_pop:
                        min_zoom = z
                        break

                if min_zoom > self.max_zoom:
                    return

                color_rgb565 = hex_to_rgb565(cfg.get('color', '#000000'))

                # Priority based on population - higher population = on top
                if population >= 500000:
                    nibble = 15  # Major cities (Paris, Lyon, Marseille, Toulouse...)
                elif population >= 100000:
                    nibble = 14  # Large cities
                elif population >= 15000:
                    nibble = 13  # Towns (Cugnaux, Muret...)
                else:
                    nibble = 12  # Small towns and villages

                # Split long names on 2 lines at hyphen or space near middle
                if len(name) > 12:
                    mid = len(name) // 2
                    best = -1
                    best_dist = len(name)
                    for i, c in enumerate(name):
                        if c in ('-', ' '):
                            dist = abs(i - mid)
                            if dist < best_dist:
                                best_dist = dist
                                best = i
                    if best > 0:
                        if name[best] == '-':
                            name = name[:best+1] + '\n' + name[best+1:]
                        else:
                            name = name[:best] + '\n' + name[best+1:]

                name_bytes = name.encode('utf-8')[:255]

                self.features.append({
                    'geom_type': GEOM_TEXT,
                    'coords': [(n.location.lon, n.location.lat)],
                    'color_rgb565': color_rgb565,
                    'zoom_priority': pack_zoom_priority(min_zoom, nibble),
                    'font_size': text_cfg['font_size'],
                    'text': name_bytes,
                    'population': population,
                })
                self.stats['text_labels'] += 1
                self.stats['nodes_processed'] += 1
                return

    def way(self, w):
        """Process way - extract roads and linear features."""
        self.stats['ways_processed'] += 1
        self._log_progress()

        # Check if this way is part of boundary relations
        if w.id in self.boundary_ways:
            coords = []
            for node in w.nodes:
                if node.location.valid():
                    coords.append((node.location.lon, node.location.lat))
            if len(coords) >= 2:
                for bnd in self.boundary_ways[w.id]:
                    self.features.append({
                        'geom_type': GEOM_LINESTRING,
                        'coords': coords,
                        'color_rgb565': bnd['color_rgb565'],
                        'zoom_priority': bnd['zoom_priority'],
                        'width_meters': 0.0,
                        'width_pixels': bnd.get('width_pixels', 1),
                        'name': w.tags.get('name', ''),
                    })
                    self.stats['boundary_ways_extracted'] += 1

        if not self._has_interesting_tags(w.tags):
            self.stats['features_filtered'] += 1
            return

        tags = self._tags_to_dict(w.tags)

        if not self._is_feature_in_config(tags):
            self.stats['features_filtered'] += 1
            return

        layer = get_layer_for_tags(tags)
        if layer is None:
            self.stats['features_filtered'] += 1
            return

        min_zoom = get_zoom_for_tags(tags, self.config)
        if min_zoom > self.max_zoom:
            self.stats['features_filtered'] += 1
            return

        coords = []
        for node in w.nodes:
            if node.location.valid():
                coords.append((node.location.lon, node.location.lat))

        if len(coords) < 2:
            self.stats['features_filtered'] += 1
            return

        # Railway service tracks (yard, siding, spur): push to z13 per OSM Carto
        if 'railway' in tags and tags.get('service') in ('yard', 'siding', 'spur', 'crossover'):
            min_zoom = max(min_zoom, 13)
            if min_zoom > self.max_zoom:
                self.stats['features_filtered'] += 1
                return

        is_closed = len(coords) >= 4 and coords[0] == coords[-1]
        
        # Tags that automatically qualify a closed way as an area/polygon
        area_qualifiers = {
            'building', 'landuse', 'water', 'amenity', 'leisure', 'natural',
            'waterway', 'man_made', 'aeroway', 'historic', 'military'
        }
        
        has_area_tag = any(k in tags for k in area_qualifiers)
        # Explicitly force area for specific water body tags
        if tags.get('natural') == 'bay' or \
           tags.get('landuse') == 'reservoir' or \
           tags.get('waterway') == 'riverbank':
            has_area_tag = True
        is_area_tags = is_closed and (has_area_tag or tags.get('area') == 'yes')

        color = get_color_for_tags(tags, self.config)
        color_rgb565 = hex_to_rgb565(color)

        if is_closed and is_area_tags and 'highway' not in tags:
            # This logic will be handled by area(), but we might catch some here.
            # Assign a polygon nibble just in case.
            nibble = 3 if layer == 'water' else 2
            
            subclass = tags.get('natural', '') or tags.get('landuse', '') or tags.get('leisure', '')

            feature = {
                'id': w.id,
                'geom_type': GEOM_POLYGON,
                'coords': coords,
                'color_rgb565': color_rgb565,
                'zoom_priority': pack_zoom_priority(min_zoom, nibble),
                'width_meters': 0.0,  # Polygons don't use width
                'subclass': subclass,  # Store for merge logic
                'name': tags.get('name', ''),
            }
            self.features.append(feature)
            self.stats['features_extracted'] += 1
            self.processed_way_ids.add(w.id)
            return

        # Extract width in meters for roads/railways/waterways
        width_meters = 0.0
        if any(tag in tags for tag in WIDTH_TAGS):
            width_meters = self._get_width_meters(tags)

        # Do not draw centerlines for wide water bodies (polygons should be used instead)
        if layer == 'water' and width_meters >= 2.0:
            self.stats['features_filtered'] += 1
            return

        # Store line type for zoom-based width lookup
        highway_type = tags.get('highway', '') or tags.get('railway', '') or tags.get('aeroway', '')
        ref = tags.get('ref', '')
        old_ref = tags.get('old_ref', '')
        name = tags.get('name', '')

        # Fixed Z-order (nibble) for rendering priority (8-15: Structure)
        priority_map = {
            # Z=14: Railways (above all roads for level crossings priority)
            'rail': 14, 'subway': 14, 'tram': 14, 'light_rail': 14,
            'narrow_gauge': 14, 'funicular': 14, 'monorail': 14,
            # Z=13: Major roads & motorways
            'motorway': 13, 'trunk': 13, 'primary': 13,
            # Z=12: Secondary roads
            'secondary': 12, 'tertiary': 12,
            # Z=11: Residential and minor roads
            'residential': 11, 'unclassified': 11, 'living_street': 11, 'pedestrian': 11,
            # Z=9-10: Links/ramps differentiated by hierarchy (below main roads but above service)
            'motorway_link': 11, 'trunk_link': 10, 'primary_link': 10, 'secondary_link': 9, 'tertiary_link': 9,
            # Z=8: Service, tracks and paths
            'service': 8, 'track': 8, 'path': 8, 'footway': 8, 'cycleway': 8
        }

        if layer == 'water':
            nibble = 5
        else:
            nibble = priority_map.get(highway_type, 8)  # Default for other minor ways

        # Log railways to verify nibble assignment
        if highway_type in ('rail', 'subway', 'tram', 'light_rail', 'narrow_gauge', 'funicular', 'monorail'):
            print(f"[RAILWAY] way={w.id}, type={highway_type}, nibble={nibble}")

        # Bridges: shift up to ensure above ALL normal roads (max normal road is 13)
        # Major bridges (roads/links): nibble+3 ensures above normal motorway (13)
        # Minor bridges (track/path): nibble+2 sufficient for hierarchy
        original_nibble = nibble
        if tags.get('bridge') in ('yes', 'viaduct'):
            if highway_type in ('track', 'path', 'footway', 'cycleway', 'bridleway'):
                nibble = min(nibble + 2, 15)  # Minor bridges: +2
            else:
                nibble = min(nibble + 3, 15)  # Major bridges: +3 to be above motorway(13)
            print(f"[BRIDGE] way={w.id}, ref={ref}, highway={highway_type}, nibble {original_nibble}→{nibble}")

        # Tunnels: shift down to ensure below ground level while preserving hierarchy
        # motorway tunnel: max(13-11,1)=2, secondary tunnel: max(12-11,1)=1, etc.
        if tags.get('tunnel') in ('yes', 'culvert'):
            nibble = max(nibble - 11, 1)  # Shift down by 11, minimum 1

        # Densify curves for smooth rendering (add intermediate points)
        # Only for roads and railways - NOT aeroways (runways/taxiways should stay straight)
        is_aeroway = tags.get('aeroway', '') != ''
        if highway_type and not is_aeroway and len(coords) >= 2:
            coords = densify_linestring(coords, max_segment_degrees=0.0001)

        feature = {
            'id': w.id,
            'geom_type': GEOM_LINESTRING,
            'coords': coords,
            'color_rgb565': color_rgb565,
            'zoom_priority': pack_zoom_priority(min_zoom, nibble),
            'width_meters': width_meters,
            'highway_type': highway_type,
            'has_ref': bool(ref),
            'ref': ref,
            'old_ref': old_ref,
            'name': name,
        }
        self.features.append(feature)
        self.stats['features_extracted'] += 1

        # Create road number label for major roads with ref
        # Display at z10: A* (autoroutes), N* (nationales), D1xxx with old_ref=N* (major former nationales)
        if ref and highway_type in ('motorway', 'trunk', 'primary', 'secondary'):
            # Filter by road number (ref), not highway_type:
            # - A* : motorways (all)
            # - N* : national roads (all)
            # - D1000-D1999 : only major former national roads with old_ref=N* (e.g., D1124 was N124)
            should_create_label = False
            if ref.startswith('A') or ref.startswith('N'):
                should_create_label = True
            elif ref.startswith('D'):
                # Extract number from D-road (e.g., "D1124" -> 1124)
                try:
                    d_number = int(ref[1:])
                    # Only D1000-D1999 (major former nationales) with old_ref=N*
                    if 1000 <= d_number <= 1999 and old_ref and old_ref.startswith('N'):
                        should_create_label = True
                except (ValueError, IndexError):
                    pass  # Invalid D-road format, skip

            if should_create_label:
                # Space out labels: only create one every 25 segments
                self.road_label_counters[ref] += 1
                if self.road_label_counters[ref] % 25 == 1:
                    # Generate 3 candidate positions (25%, 50%, 75%) for collision avoidance
                    candidates = []
                    for ratio in [0.25, 0.5, 0.75]:
                        idx = int(len(coords) * ratio)
                        candidates.append(coords[idx])

                    ref_label = {
                        'geom_type': GEOM_TEXT,
                        'coords': [candidates[1]],
                        'coords_candidates': candidates,
                        'color_rgb565': darken_rgb565(color_rgb565),  # Text: dark
                        'bg_color_rgb565': lighten_rgb565(color_rgb565),  # Background: light
                        'border_color_rgb565': color_rgb565,  # Border: original
                        'zoom_priority': pack_zoom_priority(10, 98),
                        'font_size': 2,
                        'text': ref.encode('utf-8')[:32],
                        'population': 0,
                    }
                    self.features.append(ref_label)
                    self.stats['features_extracted'] += 1

    def _get_width_meters(self, tags: Dict[str, str]) -> float:
        """Extract width in meters from OSM tags.

        Priority:
        1. width=* tag (meters)
        2. lanes=* tag (lanes × 3.5m)
        3. Return 0 (will become 1 pixel default)
        """
        # Check for explicit width tag
        if 'width' in tags:
            try:
                width_str = tags['width'].replace('m', '').replace(' ', '').strip()
                return float(width_str)
            except (ValueError, TypeError):
                pass

        # Check for lanes tag
        if 'lanes' in tags:
            try:
                lanes = int(tags['lanes'])
                return lanes * 3.5  # Standard lane width
            except (ValueError, TypeError):
                pass

        return 0.0
    
    def area(self, a):
        """Process area - handles multipolygon relations."""
        self.stats['areas_processed'] += 1
        self._log_progress()

        # DEBUG: Check if area() is called
        if self.stats['areas_processed'] <= 5:
            print(f"[DEBUG] area() called! id={a.id}, area_count={self.stats['areas_processed']}")

        tags = self._tags_to_dict(a.tags)

        # DEBUG: Trace water areas
        if tags.get('natural') == 'water' or tags.get('waterway') == 'riverbank':
            print(f"[DEBUG AREA] Water area id={a.id}, tags={tags}")

        if tags.get('natural') == 'grassland':
            print(f"[GRASSLAND] Area id={a.id}, tags={tags}")

        # Skip boundary relations
        if tags.get('boundary') == 'administrative':
            self.stats['area_boundary'] += 1
            return

        # Check if feature is in config and has a layer mapping
        if not self._is_feature_in_config(tags):
            self.stats['area_no_config'] += 1
            print(f"[DEBUG REJECT] Area id={a.id} rejected: not in config, tags={tags}")
            return

        layer = get_layer_for_tags(tags)
        if layer is None:
            self.stats['area_no_layer'] += 1
            return

        # Force water layer identity for correct hole processing (islands)
        if (tags.get('natural') == 'water' or
            tags.get('natural') == 'bay' or
            tags.get('waterway') == 'riverbank' or
            tags.get('landuse') == 'reservoir'):
            layer = 'water'

        # Force buildings layer for any polygon with building tag
        # This ensures hangars (aeroway=hangar + building=yes/hangar) render
        # as buildings (#d9d0c9 beige) not as aeroway infrastructure (#dadae0 grey)
        # Per OSM wiki: hangars have both aeroway=hangar and building=* tags
        if 'building' in tags:
            layer = 'buildings'

        # Removed the 'highway in tags' filter that was causing issues

        min_zoom = get_zoom_for_tags(tags, self.config)
        if min_zoom > self.max_zoom:
            self.stats['area_zoom_filtered'] += 1
            return

        # Construction de la géométrie
        try:
            wkb = self.wkbfab.create_multipolygon(a)
            geom = shapely.wkb.loads(wkb, hex=True)
            if not geom.is_valid:
                geom = geom.buffer(0)
                if geom.is_empty:
                    self.stats['area_exception'] += 1
                    return

            # Fixed Z-order (nibble) for polygon layers (0-7: Scenery & Buildings)
            layer_to_nibble = {
                'aeroways': 1,                   # Z=1: Airport base
                'landuse': 2, 'terrain': 2,      # Z=2: Landcover (residential, forest, farmland)
                'water': 3,                      # Z=3: All water bodies
                'leisure': 4, 'amenities': 4,    # Z=4: Parks and amenities
                'surface': 5,                    # Z=5: Ground cover (grass, meadow) inside leisure zones
                'parking': 5,                    # Z=5: Parking lots inside leisure zones
                'infrastructure': 6,
                'buildings': 7                   # Z=7: Buildings (above infrastructure)
            }
            nibble = layer_to_nibble.get(layer, 2)

            # leisure=track renders above other leisure polygons (sports_centre background)
            if tags.get('leisure') == 'track':
                nibble = 6

            color = get_color_for_tags(tags, self.config)
            color_rgb565 = hex_to_rgb565(color)
            
            # Force water color to ensure consistency, overriding JSON
            if layer == 'water':
                color_rgb565 = hex_to_rgb565("#aad3df")

            # Extract subclass for landcover discrimination (wood/forest vs farmland)
            subclass = tags.get('natural', '') or tags.get('landuse', '') or tags.get('leisure', '')

            polygons = []
            if geom.geom_type == 'Polygon':
                polygons = [geom]
            elif geom.geom_type == 'MultiPolygon':
                polygons = list(geom.geoms)

            for poly in polygons:
                if poly.is_empty or not poly.exterior:
                    continue
                coords = list(poly.exterior.coords)
                if len(coords) < 4:
                    continue

                inner_rings = []
                if poly.interiors:
                    for interior in poly.interiors:
                        if len(interior.coords) >= 4:
                            inner_rings.append(list(interior.coords))

                feature_data = {
                    'geom_type': GEOM_POLYGON,
                    'coords': coords,
                    'color_rgb565': color_rgb565,
                    'zoom_priority': pack_zoom_priority(min_zoom, nibble),
                    'width_meters': 0.0,
                    'inner_rings': inner_rings,
                    'subclass': subclass,  # Store for merge logic
                    'layer': layer,  # Store layer name for inner_rings handling
                    'name': tags.get('name', ''),
                }

                # DEBUG: Trace large water polygons
                if layer == 'water' and len(coords) > 20:
                    print(f"[EXTRACT] Water polygon: pts={len(coords)}, holes={len(inner_rings)}, subclass={subclass}, rgb565=0x{color_rgb565:04x}")

                self.features.append(feature_data)
                self.stats['features_extracted'] += 1
        except Exception as e:
            self.stats['area_exception'] += 1
            # Debug: log first 10 errors
            if self.stats['area_exception'] <= 10:
                logger.warning(f"Area extraction failed: {e} | tags: {tags}")


def densify_linestring(coords: List[Tuple[float, float]], max_segment_degrees: float) -> List[Tuple[float, float]]:
    """Add intermediate points to linestring for smoother curves."""
    if len(coords) <= 1 or not SHAPELY_AVAILABLE:
        return coords

    try:
        from shapely.geometry import LineString
        from shapely.ops import segmentize
        line = LineString(coords)
        # Segmentize adds points so no segment is longer than max_segment_degrees
        densified = segmentize(line, max_segment_degrees)
        return list(densified.coords)
    except Exception:
        return coords


def simplify_coords(coords: List[Tuple[float, float]], tolerance: float) -> List[Tuple[float, float]]:
    """Simple Douglas-Peucker-like simplification."""
    if len(coords) <= 2:
        return coords

    # Use shapely for simplification if available
    if SHAPELY_AVAILABLE:
        from shapely.geometry import LineString
        line = LineString(coords)
        simplified = line.simplify(tolerance, preserve_topology=True)
        return list(simplified.coords)

    return coords


def write_nav_tile(features: List[Dict], output_path: str, zoom: int, tile_x: int, tile_y: int, tolerance: float) -> bool:
    """
    Write features to NAV binary tile format using relative coordinates.
    """
    # Calculate tile bounds
    n = 2.0 ** zoom
    lon_deg_per_tile = 360.0 / n
    tile_min_lon = -180.0 + tile_x * lon_deg_per_tile
    tile_max_lon = tile_min_lon + lon_deg_per_tile
    
    def lat_to_merc(l):
        r = math.radians(l)
        r = max(-0.999 * math.pi / 2, min(0.999 * math.pi / 2, r))
        return math.log(math.tan(r) + (1.0 / math.cos(r)))

    def lat_from_tile_y(y, z):
        n = 2.0 ** z
        lat_rad = math.atan(math.sinh(math.pi * (1 - 2 * y / n)))
        return math.degrees(lat_rad)

    tile_max_lat = lat_from_tile_y(tile_y, zoom)
    tile_min_lat = lat_from_tile_y(tile_y + 1, zoom)
    
    t_max_merc = lat_to_merc(tile_max_lat)
    t_min_merc = lat_to_merc(tile_min_lat)
    merc_range = t_max_merc - t_min_merc

    # Clipping box with margins: 10% for polygons, 100% for linestrings (long runways)
    poly_margin = 0.10  # Small margin for polygons to avoid artifacts
    line_margin = 1.0   # 100% = 1 full tile margin for runways spanning 8-12 tiles

    poly_lon_margin = (tile_max_lon - tile_min_lon) * poly_margin
    poly_lat_margin = (tile_max_lat - tile_min_lat) * poly_margin
    line_lon_margin = (tile_max_lon - tile_min_lon) * line_margin
    line_lat_margin = (tile_max_lat - tile_min_lat) * line_margin

    clip_box = None
    clip_box_line = None
    if SHAPELY_AVAILABLE:
        from shapely.geometry import box, Polygon, MultiPolygon, LineString, MultiLineString, GeometryCollection
        clip_box = box(tile_min_lon - poly_lon_margin, tile_min_lat - poly_lat_margin,
                       tile_max_lon + poly_lon_margin, tile_max_lat + poly_lat_margin)
        clip_box_line = box(tile_min_lon - line_lon_margin, tile_min_lat - line_lat_margin,
                            tile_max_lon + line_lon_margin, tile_max_lat + line_lat_margin)

    os.makedirs(os.path.dirname(output_path), exist_ok=True)

    # Merge polygons of the same style to reduce feature count
    if SHAPELY_AVAILABLE:
        from shapely.geometry import Polygon as ShapelyPolygon, MultiPolygon as ShapelyMultiPolygon
        from shapely.ops import unary_union as shapely_unary_union

        # Area filter thresholds (applied to ALL polygons before grouping)
        # OpenMapTiles formula with zoom-adapted multipliers
        min_area_deg2 = 0.0
        if zoom < 14:
            zres_prev = 360.0 / (2**(zoom - 1) * 256)
            if zoom <= 7:
                multiplier = 2.5
            elif zoom == 8:
                multiplier = 1.8  # z8 : très permissif pour voir plus de landuse
            elif zoom == 9:
                multiplier = 2.5  # z9 : garde le bon niveau actuel
            else:
                multiplier = 3.0
            min_area_deg2 = (zres_prev ** 2) * multiplier

        polygons_by_style = defaultdict(list)
        other_features = []
        filtered_by_area = 0

        for feat in features:
            if feat['geom_type'] == GEOM_POLYGON:
                # Apply area filter to ALL polygons (not just grouped ones)
                if min_area_deg2 > 0:
                    inner = feat.get('inner_rings', [])
                    if inner:
                        sp = ShapelyPolygon(feat['coords'], inner)
                    else:
                        sp = ShapelyPolygon(feat['coords'])
                    if sp.area < min_area_deg2:
                        # DEBUG: Trace large water polygons being filtered
                        if feat.get('layer') == 'water' and len(feat['coords']) > 20:
                            print(f"[FILTER_AREA z{zoom}] Water polygon FILTERED: pts={len(feat['coords'])}, area={sp.area:.9f} < min={min_area_deg2:.9f}")
                        filtered_by_area += 1
                        continue  # Skip small polygons

                # Group by color, priority AND subclass to separate wood/forest from farmland
                subclass = feat.get('subclass', '')
                style_key = (feat['color_rgb565'], feat['zoom_priority'], subclass)
                polygons_by_style[style_key].append(feat)
            else:
                other_features.append(feat)

        if filtered_by_area > 0:
            logger.debug(f"  Tile {tile_x},{tile_y}: Filtered {filtered_by_area} polygons by area at z{zoom}")

        merged_features = []
        merge_stats = {'holes_total': 0, 'holes_removed': 0, 'sharding_fallbacks': 0, 'groups_merged': 0}
        for (color, priority, subclass), poly_list in polygons_by_style.items():
            if len(poly_list) < 2:
                merged_features.extend(poly_list)
                continue
            try:
                shapely_polys = []
                for p in poly_list:
                    inner = p.get('inner_rings', [])
                    if inner:
                        sp = ShapelyPolygon(p['coords'], inner)
                    else:
                        sp = ShapelyPolygon(p['coords'])
                    if not sp.is_valid:
                        sp = sp.buffer(0)
                    if not sp.is_empty:
                        shapely_polys.append(sp)

                if not shapely_polys:
                    merged_features.extend(poly_list)
                    continue

                pixel_deg = 360.0 / (2**zoom * 256)

                # Extract priority nibble from packed byte (zoom_priority = zoom<<4 | prio)
                priority_nibble = priority & 0x0F
                # landuse(1-2), terrain(2-3) only — NOT water(4-5) to avoid flooding
                is_landcover = priority_nibble <= 3

                # OpenMapTiles-style merge: only wood/forest, keep farmland/grass individual
                should_merge = is_landcover and subclass in ('wood', 'forest')

                if should_merge:
                    # Merge wood/forest to reduce fragmentation
                    merged = shapely_unary_union(shapely_polys)
                    # Simplify merged result (merge creates complex polygons with too many vertices)
                    merged = merged.simplify(pixel_deg * 0.5, preserve_topology=True)
                else:
                    # Keep individual: farmland, grass, water, roads
                    merged_features.extend(poly_list)
                    continue

                parts = []
                if isinstance(merged, ShapelyMultiPolygon):
                    parts = list(merged.geoms)
                elif isinstance(merged, ShapelyPolygon):
                    parts = [merged]

                min_hole_deg2 = (pixel_deg ** 2) * K_VISIBILITY * K_HOLE_FACTOR
                total_merged_points = 0

                # Get layer from first feature in group
                feature_layer = poly_list[0].get('layer', '')

                candidate_features = []
                for part in parts:
                    if not part.is_empty and part.exterior and len(part.exterior.coords) >= 4:
                        # Keep inner_rings for water (islands), strip for landcover (pitting)
                        if feature_layer == 'water':
                            inner_rings = [list(interior.coords) for interior in part.interiors if len(interior.coords) >= 4]
                            for interior in part.interiors:
                                merge_stats['holes_total'] += 1
                        else:
                            inner_rings = []
                            for interior in part.interiors:
                                merge_stats['holes_total'] += 1
                                merge_stats['holes_removed'] += 1

                        ext_coords = list(part.exterior.coords)
                        pt_count = len(ext_coords)
                        total_merged_points += pt_count
                        candidate_features.append({
                            'geom_type': GEOM_POLYGON,
                            'coords': ext_coords,
                            'inner_rings': inner_rings,
                            'color_rgb565': color,
                            'zoom_priority': priority,
                            'width_meters': 0.0,
                            'subclass': subclass,  # Preserve subclass after merge
                            'layer': feature_layer  # Preserve layer
                        })

                if total_merged_points > 65535:
                    # Merge too complex, keep original separate polygons
                    merge_stats['sharding_fallbacks'] += 1
                    merged_features.extend(poly_list)
                else:
                    merge_stats['groups_merged'] += 1
                    merged_features.extend(candidate_features)
            except Exception:
                merged_features.extend(poly_list)

        features = other_features + merged_features
        logger.debug(f"  Tile {tile_x},{tile_y}: Merge: {merge_stats['groups_merged']} groups merged, "
                     f"{merge_stats['holes_removed']}/{merge_stats['holes_total']} holes removed, "
                     f"{merge_stats['sharding_fallbacks']} sharding fallbacks")

    # DEBUG: Check features entering the tile
    debug_tiles = [(11962, [16513, 16514, 16515]), (11966, [16509, 16510, 16511])]
    debug_roads = ['Boulevard Silvio Trentin', 'Boulevard Pierre et Marie Curie', 'Avenue de Lardenne']

    for debug_y, debug_xs in debug_tiles:
        if tile_y == debug_y and tile_x in debug_xs:
            print(f"[DEBUG ENTER] Tile {tile_x},{tile_y}: Start writing. Features count={len(features)}")
            if features:
                 print(f"[DEBUG ENTER] Feature 0 keys: {list(features[0].keys())}")

            for road_name in debug_roads:
                road_count = sum(1 for f in features if road_name.lower() in f.get('name', '').lower())
                if road_count > 0:
                    print(f"  [DEBUG ENTER] '{road_name}' count: {road_count}")
                    # Print one instance
                    for f in features:
                        if road_name.lower() in f.get('name', '').lower():
                             print(f"  [DEBUG ENTER] Found {road_name}: id={f.get('id')}, coords_len={len(f.get('coords', []))}")
                             break

    # Final sort by priority nibble to ensure strict rendering order on device.
    # This is the most critical step for correct Z-ordering.
    features.sort(key=lambda f: f['zoom_priority'] & 0x0F)

    written_features = 0
    filtered_by_size = 0
    filtered_holes_write = 0
    total_holes_write = 0
    with open(output_path, 'wb') as f:
        f.write(struct.pack('<4sHiiii', NAV_MAGIC, 0,
                           int(tile_min_lon * COORD_SCALE),
                           int(tile_min_lat * COORD_SCALE),
                           int(tile_max_lon * COORD_SCALE),
                           int(tile_max_lat * COORD_SCALE)))

        # Background land polygon covering the entire tile
        bg_points = [(0, 0), (4096, 0), (4096, 4096), (0, 4096), (0, 0)]
        f.write(struct.pack('<B', GEOM_POLYGON))       # type
        f.write(struct.pack('<H', hex_to_rgb565(LAND_BG_COLOR)))  # color
        f.write(struct.pack('<B', pack_zoom_priority(0, 0)))  # lowest priority (Z=0)
        f.write(struct.pack('<B', 1))                   # width
        f.write(struct.pack('<BBBB', 0, 0, 255, 255))  # bbox = full tile
        f.write(struct.pack('<H', 5))                   # 5 points
        f.write(b'\x00')                                # reserved
        for px, py in bg_points:
            f.write(struct.pack('<hh', px, py))
        f.write(struct.pack('<H', 1))                   # 1 ring
        f.write(struct.pack('<H', 5))                   # ring end at point 5
        written_features += 1

        for feature in features:
            if written_features >= 65534:
                logger.warning(f"  Tile {tile_x},{tile_y} z{zoom}: HIT FEATURE LIMIT (65534)! Truncating rest of tile.")
                break
            # DEBUG: Trace specific roads at start of loop
            debug_tiles_check = [(11962, [16513, 16514, 16515]), (11966, [16509, 16510, 16511])]
            debug_roads_check = ['Boulevard Silvio Trentin', 'Boulevard Pierre et Marie Curie', 'Avenue de Lardenne']

            for debug_y, debug_xs in debug_tiles_check:
                if tile_y == debug_y and tile_x in debug_xs:
                    for road_name in debug_roads_check:
                        if road_name.lower() in feature.get('name', '').lower():
                            print(f"[DEBUG START] Tile {tile_x},{tile_y}: Processing {feature.get('name')}. Pts={len(feature['coords'])}, Geom={feature['geom_type']}, Layer={feature.get('layer', 'N/A')}")
                            print(f"  [DEBUG START] SHAPELY_AVAILABLE={SHAPELY_AVAILABLE}")
                            break

            # Handle text features separately
            if feature['geom_type'] == GEOM_TEXT:
                lon, lat = feature['coords'][0]
                px = int((lon - tile_min_lon) / (tile_max_lon - tile_min_lon) * 4096)
                m_y = lat_to_merc(lat)
                py = int((t_max_merc - m_y) / merc_range * 4096)

                if not (-8192 < px < 12288 and -8192 < py < 12288):
                    continue

                text_bytes = feature['text']
                text_len = len(text_bytes)
                has_shield = 'bg_color_rgb565' in feature
                # data_size: x,y + text_len + text + (shield colors if present)
                data_size = 4 + 1 + text_len + (4 if has_shield else 0)
                coord_count = (data_size + 3) // 4
                padded_size = coord_count * 4

                bx = max(0, min(255, px >> 4))
                by = max(0, min(255, py >> 4))

                # Header
                f.write(struct.pack('<B', GEOM_TEXT))
                f.write(struct.pack('<H', feature['color_rgb565']))
                f.write(struct.pack('<B', feature['zoom_priority']))
                f.write(struct.pack('<B', feature.get('font_size', 0)))
                f.write(struct.pack('<BBBB', bx, by, bx, by))
                f.write(struct.pack('<H', coord_count))
                f.write(struct.pack('<B', 1 if has_shield else 0))  # Shield flag

                # Data: position + text + shield colors
                f.write(struct.pack('<hh', px, py))
                f.write(struct.pack('<B', text_len))
                f.write(text_bytes)
                if has_shield:
                    f.write(struct.pack('<H', feature['bg_color_rgb565']))
                    f.write(struct.pack('<H', feature['border_color_rgb565']))
                # Pad
                padding = padded_size - data_size
                if padding > 0:
                    f.write(b'\x00' * padding)

                written_features += 1
                continue

            orig_coords = feature['coords']
            inner_rings = feature.get('inner_rings', [])
            is_polygon = feature['geom_type'] == GEOM_POLYGON

            feature_layer = feature.get('layer', '')

            # DEBUG: Check layer for large polygons
            if is_polygon and len(orig_coords) > 100:
                print(f"[WRITE] Large polygon: pts={len(orig_coords)}, layer='{feature_layer}', rgb565=0x{feature.get('color_rgb565', 0):04x}")
            if is_polygon and inner_rings and SHAPELY_AVAILABLE:
                total_holes_write += len(inner_rings)
                
                # For water, always keep holes (islands)
                if feature_layer != 'water':
                    # For other layers, filter holes by visible area at the current zoom
                    pixel_deg = 360.0 / (2**zoom * 256)
                    
                    # More permissive for z13+ to avoid "blob" effect on residential areas
                    min_hole_pixels_sq = K_VISIBILITY * 1.5 if zoom >= 13 else K_VISIBILITY * 10.0
                    min_hole_area_deg2 = (pixel_deg ** 2) * min_hole_pixels_sq

                    filtered_inner_rings = []
                    for interior in inner_rings:
                        try:
                            hole_poly = Polygon(interior)
                            if hole_poly.area >= min_hole_area_deg2:
                                filtered_inner_rings.append(interior)
                            else:
                                filtered_holes_write += 1
                        except Exception:
                            # Invalid hole geometry, discard
                            filtered_holes_write += 1
                    
                    inner_rings = filtered_inner_rings

            # Each entry will be a list of rings: [ [ext_pts], [hole1_pts], ... ]
            final_features_data = []

            # Clip geometry (polygons with small margin, linestrings with large margin)
            active_clip_box = clip_box_line if (not is_polygon and clip_box_line) else clip_box
            if active_clip_box:
                try:
                    from shapely.geometry import Polygon, MultiPolygon, LineString, MultiLineString, GeometryCollection
                    
                    # 1. Create the appropriate Shapely geometry
                    if is_polygon:
                        if inner_rings:
                            geom = Polygon(orig_coords, inner_rings)
                        else:
                            geom = Polygon(orig_coords)
                        # Only repair Polygons (buffer(0) fixes self-intersections)
                        if not geom.is_valid:
                            geom = geom.buffer(0)
                    else:
                        # For roads/lines, NEVER use buffer(0) as it destroys the geometry
                        if len(orig_coords) < 2:
                            continue
                        geom = LineString(orig_coords)

                    if geom is None or geom.is_empty:
                        continue

                    # DEBUG: Before clipping
                    debug_roads_clip = ['Boulevard Silvio Trentin', 'Boulevard Pierre et Marie Curie', 'Avenue de Lardenne']
                    debug_tiles_clip = [(11962, [16513, 16514, 16515]), (11966, [16509, 16510, 16511])]
                    is_debug_road = any(road.lower() in feature.get('name', '').lower() for road in debug_roads_clip)
                    is_debug_tile = any(tile_y == debug_y and tile_x in debug_xs for debug_y, debug_xs in debug_tiles_clip)
                    if is_debug_road and is_debug_tile:
                        print(f"[DEBUG CLIP BEFORE] Tile {tile_x},{tile_y}: {feature.get('name')} (id={feature.get('id')}), geom_type={type(geom).__name__}, pts={len(orig_coords)}, is_valid={geom.is_valid}")

                    # 2. Perform clipping (intersection with the tile bounding box)
                    clipped = geom.intersection(active_clip_box)

                    # DEBUG: After clipping
                    if is_debug_road and is_debug_tile:
                        if clipped.is_empty:
                            print(f"[DEBUG CLIP DROPPED] Tile {tile_x},{tile_y}: {feature.get('name')} (id={feature.get('id')}) - clipped result is EMPTY!")
                        else:
                            print(f"[DEBUG CLIP AFTER] Tile {tile_x},{tile_y}: {feature.get('name')} (id={feature.get('id')}), result_type={type(clipped).__name__}, has_coords={hasattr(clipped, 'coords')}")

                    if clipped.is_empty:
                        continue

                    # 3. Extract parts from the result (handles MultiLineStrings and GeometryCollections)
                    parts = []
                    if isinstance(clipped, GeometryCollection):
                        parts = list(clipped.geoms)
                    else:
                        parts = [clipped]

                    for part in parts:
                        if is_polygon:
                            # Process Polygon results
                            if isinstance(part, (Polygon, MultiPolygon)):
                                polys = [part] if isinstance(part, Polygon) else list(part.geoms)
                                for p in polys:
                                    if not p.is_empty and p.exterior and len(p.exterior.coords) >= 4:
                                        # Simplify polygons depending on zoom level
                                        if feature_layer in ('landuse', 'terrain') and zoom < 14:
                                            simplified_poly = p.simplify(tolerance, preserve_topology=True)
                                        else:
                                            simplified_poly = p
                                            
                                        if simplified_poly.is_empty or not simplified_poly.exterior:
                                            continue
                                            
                                        rings = [list(simplified_poly.exterior.coords)]
                                        for interior in simplified_poly.interiors:
                                            if len(interior.coords) >= 4:
                                                rings.append(list(interior.coords))
                                        final_features_data.append(rings)
                        else:
                            # Process LineString results (Roads, Rivers, etc.)
                            lines = []
                            if isinstance(part, LineString):
                                lines = [part]
                            elif isinstance(part, MultiLineString):
                                lines = list(part.geoms)
                            
                            for l in lines:
                                if len(l.coords) < 2:
                                    continue
                                
                                # FIX: Removed hardcoded 0.25 (which meant 27km in degrees)
                                # We keep full detail for roads and water, or use tile-relative tolerance
                                if feature_layer in ('water', 'roads'):
                                    simplified = l
                                elif feature_layer == 'infrastructure':
                                    # Use a microscopic tolerance for noise removal at high zooms
                                    pixel_deg = 360.0 / (2**zoom * 256)
                                    simplified = l.simplify(pixel_deg * 0.1, preserve_topology=True)
                                else:
                                    simplified = l.simplify(tolerance, preserve_topology=True)
                                    
                                if len(simplified.coords) >= 2:
                                    final_features_data.append([list(simplified.coords)])
                                    # DEBUG: Confirm road added to final_features_data
                                    if is_debug_road and is_debug_tile:
                                        print(f"[DEBUG ADDED] Tile {tile_x},{tile_y}: {feature.get('name')} (id={feature.get('id')}) ADDED to final_features_data, pts={len(simplified.coords)}")
                                    
                except Exception as e:
                    # Fallback: if clipping fails, use original coordinates to avoid data loss
                    if is_polygon:
                        final_features_data.append([orig_coords] + inner_rings)
                    else:
                        final_features_data.append([orig_coords])
            else:
                # No clipping active: use the original geometry
                if is_polygon and inner_rings:
                    final_features_data = [[orig_coords] + inner_rings]
                else:
                    final_features_data = [[orig_coords]]

            # DEBUG: Check if roads made it to final_features_data
            if is_debug_road and is_debug_tile and len(final_features_data) > 0:
                print(f"[DEBUG FINAL] Tile {tile_x},{tile_y}: {feature.get('name')} (id={feature.get('id')}) has {len(final_features_data)} feature parts to write")

            # Project and write the features
            for feature_rings in final_features_data:
                # Project all rings for this feature part
                projected_rings = []
                total_points = 0
                f_min_x, f_min_y = 4096, 4096
                f_max_x, f_max_y = 0, 0
                is_visible = False

                # DEBUG: Check if this is a target road for coordinate logging
                debug_roads_proj = ['Boulevard Silvio Trentin', 'Boulevard Pierre et Marie Curie', 'Avenue de Lardenne']
                debug_tiles_proj = [(11962, [16513, 16514, 16515]), (11966, [16509, 16510, 16511])]
                is_debug_road_proj = any(road.lower() in feature.get('name', '').lower() for road in debug_roads_proj)
                is_debug_tile_proj = any(tile_y == debug_y and tile_x in debug_xs for debug_y, debug_xs in debug_tiles_proj)

                for ring in feature_rings:
                    projected_ring = []
                    for lon, lat in ring:
                        px = int((lon - tile_min_lon) / (tile_max_lon - tile_min_lon) * 4096)
                        m_y = lat_to_merc(lat)
                        py = int((t_max_merc - m_y) / merc_range * 4096)

                        # DEBUG: Log projected coordinates for target roads
                        if is_debug_road_proj and is_debug_tile_proj and len(projected_ring) == 0:  # First point only
                            in_range = -8192 < px < 12288 and -8192 < py < 12288
                            print(f"[DEBUG PROJ] Tile {tile_x},{tile_y}: {feature.get('name')} (id={feature.get('id')}), first_point: lon={lon:.6f}, lat={lat:.6f} → px={px}, py={py}, in_range={in_range}")

                        if -8192 < px < 12288 and -8192 < py < 12288:
                            is_visible = True

                        projected_ring.append((px, py))
                        
                        c_px, c_py = max(0, min(4096, px)), max(0, min(4096, py))
                        f_min_x, f_min_y = min(f_min_x, c_px), min(f_min_y, c_py)
                        f_max_x, f_max_y = max(f_max_x, c_px), max(f_max_y, c_py)
                    
                    if len(projected_ring) >= (3 if is_polygon else 2):
                        projected_rings.append(projected_ring)
                        total_points += len(projected_ring)

                if is_polygon:
                    pixel_area = (f_max_x - f_min_x) * (f_max_y - f_min_y) / (16 * 16)

                    # DEBUG: Track water polygon projection
                    if feature_layer == 'water' and total_points > 20:
                        # Get lon/lat bounds from original coords
                        first_ring = feature_rings[0] if feature_rings else []
                        if first_ring:
                            lons = [lon for lon, lat in first_ring]
                            lats = [lat for lon, lat in first_ring]
                            lon_range = max(lons) - min(lons)
                            lat_range = max(lats) - min(lats)
                            # Get projected bounds
                            first_proj = projected_rings[0] if projected_rings else []
                            if first_proj:
                                pxs = [px for px, py in first_proj]
                                pys = [py for px, py in first_proj]
                                px_range = max(pxs) - min(pxs)
                                py_range = max(pys) - min(pys)
                                print(f"[PROJ z{zoom}] Water {tile_x}/{tile_y}: pts={total_points}, "
                                      f"lon_range={lon_range:.6f}°, lat_range={lat_range:.6f}°, "
                                      f"px_range={px_range}, py_range={py_range}, "
                                      f"pixel_area={pixel_area:.1f}px²")

                    # Do NOT filter water by pixel area - keep all river segments
                    if feature_layer != 'water':
                        if zoom <= 7:
                            min_area = K_VISIBILITY * 8
                        elif zoom == 8:
                            min_area = K_VISIBILITY * 6  # z8 : très permissif (12 pixels²)
                        elif zoom == 9:
                            min_area = K_VISIBILITY * 8  # z9 : garde le bon niveau actuel
                        elif zoom <= 11:
                            min_area = K_VISIBILITY * 8
                        elif zoom == 12:
                            min_area = K_VISIBILITY * 5
                        elif zoom == 13:
                            min_area = K_VISIBILITY * 2
                        elif zoom == 14:
                            min_area = K_VISIBILITY * 0.5
                        else:  # z15-16
                            min_area = K_VISIBILITY * 0.1  # 0.2 px² - capture everything
                        if pixel_area < min_area:
                            filtered_by_size += 1
                            continue

                # Hard limit: skip features exceeding uint16 capacity
                # Impossible to render on ESP32 and would corrupt binary format
                if total_points > 65535:
                    logger.warning(f"  Tile {tile_x},{tile_y} z{zoom}: SKIPPING feature with {total_points} points (limit 65535). Type={feature.get('geom_type')}")
                    continue

                width_pixels = feature.get('width_pixels', 0)
                if width_pixels == 0:
                    # Use fixed road width table for highways
                    hw_type = feature.get('highway_type', '')
                    if hw_type and hw_type in LINE_WIDTH_PER_ZOOM:
                        width_pixels = LINE_WIDTH_PER_ZOOM[hw_type].get(zoom, 1)
                    else:
                        width_meters = feature.get('width_meters', 0.0)
                        width_pixels = meters_to_pixels(width_meters, zoom) if width_meters > 0 else 1

                # Mark roads that need casing (border rendering) based on priority nibble
                priority_nibble = feature['zoom_priority'] & 0x0F
                needs_casing = priority_nibble in (12, 13)

                # Encode width with casing flag
                width_byte = min(width_pixels, 127)  # Clamp to 7 bits
                if needs_casing:
                    width_byte |= 0x80  # Set bit 7

                bx1, by1 = max(0, min(255, f_min_x >> 4)), max(0, min(255, f_min_y >> 4))
                bx2, by2 = max(0, min(255, f_max_x >> 4)), max(0, min(255, f_max_y >> 4))

                # DEBUG: Confirm writing to file
                if is_debug_road_proj and is_debug_tile_proj:
                    print(f"[DEBUG WRITE] Tile {tile_x},{tile_y}: WRITING {feature.get('name')} (id={feature.get('id')}) to file, total_points={total_points}, projected_rings={len(projected_rings)}, color={feature['color_rgb565']}, width={width_pixels}, hw_type={feature.get('highway_type', 'N/A')}")

                # Feature Header
                f.write(struct.pack('<B', feature['geom_type']))
                f.write(struct.pack('<H', feature['color_rgb565']))
                f.write(struct.pack('<B', feature['zoom_priority']))
                f.write(struct.pack('<B', width_byte))  # Width + casing flag
                f.write(struct.pack('<BBBB', bx1, by1, bx2, by2))
                f.write(struct.pack('<H', total_points))
                f.write(b'\x00')

                # Points for all rings (clamp to int16 range for long runways)
                for ring in projected_rings:
                    for px, py in ring:
                        # Clamp coordinates to fit in signed 16-bit integer range
                        px_clamped = max(-32768, min(32767, px))
                        py_clamped = max(-32768, min(32767, py))
                        f.write(struct.pack('<hh', px_clamped, py_clamped))

                if is_polygon:
                    # Write ring ends (using uint16 to support > 255 rings in complex merged areas)
                    f.write(struct.pack('<H', len(projected_rings)))
                    current_end = 0
                    for ring in projected_rings:
                        current_end += len(ring)
                        f.write(struct.pack('<H', current_end))

                written_features += 1

        f.seek(4)
        f.write(struct.pack('<H', written_features))

    logger.debug(f"  Tile {tile_x},{tile_y}: Write: {written_features} features, "
                 f"{filtered_by_size} filtered by area (<{K_VISIBILITY}px²), "
                 f"{filtered_holes_write}/{total_holes_write} holes removed (<{K_VISIBILITY * K_HOLE_FACTOR}px²)")

    return True


def convert_pbf_to_nav(input_pbf: str, output_dir: str, config_file: str,
                        zoom_range: Tuple[int, int] = (6, 17)):
    """Main conversion function - generates NAV tile files."""

    logger.info(f"Loading configuration from {config_file}")
    with open(config_file, 'r') as f:
        config = json.load(f)

    os.makedirs(output_dir, exist_ok=True)

    file_size_mb = os.path.getsize(input_pbf) / (1024 * 1024)
    logger.info(f"Processing PBF file: {input_pbf} ({file_size_mb:.1f} MB)")
    logger.info(f"Zoom range: {zoom_range[0]}-{zoom_range[1]}")
    logger.info(f"Output format: NAV binary tiles (.nav)")

    start_time = time.time()

    # First pass: scan for boundary relations (fast, no locations needed)
    logger.info("Pass 1: Scanning boundary relations...")
    scanner = BoundaryScanner(config, zoom_range[1])
    scanner.apply_file(input_pbf)
    logger.info(f"  Boundary ways found: {len(scanner.boundary_ways):,}")

    # Second pass: extract all features (including multipolygon relations)
    handler = OSMHandler(config, zoom_range)
    handler.boundary_ways = scanner.boundary_ways

    area_manager = osmium.area.AreaManager()

    # AreaManager requires TWO passes:
    logger.info("Pass 2a: Scanning multipolygon relations...")
    osmium.apply(input_pbf, area_manager.first_pass_handler())

    logger.info("Pass 2b: Building areas and extracting features...")
    idx = osmium.index.create_map('flex_mem')
    nlw = osmium.NodeLocationsForWays(idx)
    nlw.apply_nodes_to_ways = True
    # Chain handlers: nlw -> handler (nodes/ways) -> area_manager.second_pass (areas)
    osmium.apply(input_pbf, nlw, handler, area_manager.second_pass_handler(handler))
    print()

    elapsed = time.time() - start_time
    logger.info(f"Processing completed in {elapsed:.2f}s")
    logger.info(f"Statistics:")
    logger.info(f"  Nodes (peaks): {handler.stats['nodes_processed'] - handler.stats['text_labels']:,}")
    logger.info(f"  Text labels (places): {handler.stats['text_labels']:,}")
    logger.info(f"  Ways processed: {handler.stats['ways_processed']:,}")
    logger.info(f"  Areas processed: {handler.stats['areas_processed']:,}")
    logger.info(f"  Boundary ways extracted: {handler.stats['boundary_ways_extracted']:,}")
    logger.info(f"  Features extracted: {handler.stats['features_extracted']:,}")
    logger.info(f"  Features filtered: {handler.stats['features_filtered']:,}")
    logger.info(f"  Area filter breakdown:")
    logger.info(f"    - Boundary admin: {handler.stats['area_boundary']:,}")
    logger.info(f"    - Not in config: {handler.stats['area_no_config']:,}")
    logger.info(f"    - No layer mapping: {handler.stats['area_no_layer']:,}")
    logger.info(f"    - Zoom filtered: {handler.stats['area_zoom_filtered']:,}")
    logger.info(f"    - Exceptions: {handler.stats['area_exception']:,}")

    logger.info("Calculating bounding box from ALL features...")
    min_lon, max_lon = 180.0, -180.0
    min_lat, max_lat = 90.0, -90.0
    feature_count = 0
    for feature in handler.features:
        feature_count += 1
        for lon, lat in feature['coords']:
            min_lon = min(min_lon, lon)
            max_lon = max(max_lon, lon)
            min_lat = min(min_lat, lat)
            max_lat = max(max_lat, lat)
    logger.info(f"  BBox from {feature_count:,} features")
    logger.info(f"  BBox: lon=[{min_lon:.4f}, {max_lon:.4f}], lat=[{min_lat:.4f}, {max_lat:.4f}]")

    logger.info("Generating NAV tile files...")

    total_tiles = 0
    total_size = 0

    for zoom in range(zoom_range[0], zoom_range[1] + 1):
        tile_features = defaultdict(list)
        tolerance = get_simplify_tolerance(zoom)

        # Phase 1: Prepare and filter features for this zoom level
        zoom_start = time.time()
        prepared_count = 0
        total_to_process = len(handler.features)

        # Collect text labels for collision detection
        text_candidates = []

        for feature in handler.features:
            min_zoom = feature['zoom_priority'] >> 4
            if min_zoom > zoom:
                continue

            # Filter secondary roads without ref at z9 (keep only numbered departmental roads)
            if zoom == 9 and feature.get('highway_type') == 'secondary' and not feature.get('has_ref'):
                continue

            # NOTE: Simplification moved AFTER clipping to avoid inter-tile gaps
            coords = feature['coords']

            if not coords:
                continue

            # Convert point features to symbol polygons
            if feature['geom_type'] == GEOM_POINT:
                lon, lat = coords[0]
                tile_width_deg = 360.0 / (2.0 ** zoom)
                pixel_deg = tile_width_deg / 256.0
                size = pixel_deg * 3  # 3 pixel radius

                shape = feature.get('shape', 'circle')
                # Correct for Mercator distortion
                lat_size = size / math.cos(math.radians(lat))

                if shape == 'triangle':
                    # Equilateral triangle: height = size * sqrt(3)/2 ≈ size * 0.866
                    h = lat_size * 0.866
                    sym_coords = [
                        (lon, lat + h * 0.667),             # top (1/3 above center)
                        (lon - size, lat - h * 0.333),      # bottom-left
                        (lon + size, lat - h * 0.333),      # bottom-right
                        (lon, lat + h * 0.667),
                    ]
                else:  # square dot (2x2 pixels)
                    s = pixel_deg  # 1 pixel
                    ls = s / math.cos(math.radians(lat))
                    sym_coords = [
                        (lon - s, lat + ls),
                        (lon + s, lat + ls),
                        (lon + s, lat - ls),
                        (lon - s, lat - ls),
                        (lon - s, lat + ls),
                    ]

                zoom_feature = {
                    'geom_type': GEOM_POLYGON,
                    'coords': sym_coords,
                    'color_rgb565': feature['color_rgb565'],
                    'zoom_priority': feature['zoom_priority'],
                    'width_meters': 0.0
                }
            elif feature['geom_type'] == GEOM_TEXT:
                zoom_feature = {
                    'geom_type': GEOM_TEXT,
                    'coords': coords,
                    'color_rgb565': feature['color_rgb565'],
                    'zoom_priority': feature['zoom_priority'],
                    'font_size': feature.get('font_size', 0),
                    'text': feature['text'],
                }
            else:
                # Create a lightweight record for this zoom
                color_rgb565 = feature['color_rgb565']
                # Override color per zoom if defined
                hw_type = feature.get('highway_type', '')
                if hw_type and hw_type in LINE_COLOR_PER_ZOOM:
                    color_override = LINE_COLOR_PER_ZOOM[hw_type].get(zoom)
                    if color_override:
                        color_rgb565 = hex_to_rgb565(color_override)

                zoom_feature = {
                    'geom_type': feature['geom_type'],
                    'coords': coords,
                    'color_rgb565': color_rgb565,
                    'zoom_priority': feature['zoom_priority'],
                    'width_meters': feature.get('width_meters', 0.0),
                    'width_pixels': feature.get('width_pixels', 0),
                    'highway_type': hw_type,
                    'inner_rings': feature.get('inner_rings', []),
                    'layer': feature.get('layer', ''),  # Preserve layer for water detection
                    'name': feature.get('name', ''),  # Preserve name for debugging
                    'id': feature.get('id', 0),  # Preserve OSM ID for debugging
                }

            # Text labels: collect for collision detection
            if zoom_feature['geom_type'] == GEOM_TEXT:
                text_candidates.append(zoom_feature)
            else:
                is_polygon = zoom_feature['geom_type'] == GEOM_POLYGON
                tiles = get_feature_tiles(zoom_feature['coords'], zoom, is_polygon)
                
                # DEBUG: Trace assignment
                debug_roads_assign = ['Boulevard Silvio Trentin', 'Boulevard Pierre et Marie Curie', 'Avenue de Lardenne']
                debug_tiles_assign = [(11962, [16513, 16514, 16515]), (11966, [16509, 16510, 16511])]

                for road_name in debug_roads_assign:
                    if road_name.lower() in feature.get('name', '').lower():
                        for debug_y, debug_xs in debug_tiles_assign:
                            relevant_tiles = [t for t in tiles if t[1] == debug_y and t[0] in debug_xs]
                            if relevant_tiles:
                                print(f"[DEBUG ASSIGN z{zoom}] {feature.get('name')} (id={feature.get('id')}) assigned to tiles: {relevant_tiles}")

                for tile in tiles:
                    tile_features[tile].append(zoom_feature)
            
            prepared_count += 1
            if prepared_count % 25000 == 0:
                print(f"\r  Zoom {zoom:2d}: Preparing features... {prepared_count:,} / {total_to_process:,}", end='', flush=True)

        # Phase 1b: Text label collision detection
        # PRIORITY: Place names (fixed) > Road labels (moveable)
        if text_candidates:
            tile_width_deg = 360.0 / (2.0 ** zoom)
            pixel_deg = tile_width_deg / 256.0
            char_w = pixel_deg * 7  # half-width per char in degrees (1.75x for actual render size)
            label_h = pixel_deg * 11  # half-height in degrees (1.375x for actual render size)

            # Separate place names from road labels
            place_names = [f for f in text_candidates if 'coords_candidates' not in f]
            road_labels = [f for f in text_candidates if 'coords_candidates' in f]

            # STEP 1: Place names - sorted by population, place if no visual overlap
            # Population hierarchy: highest population wins in case of label collision
            place_names.sort(key=lambda f: -f.get('population', 0))

            placed_boxes = []
            places_placed = 0
            places_dropped_overlap = 0

            for pf in place_names:
                text_len = len(pf['text'])
                half_w = char_w * text_len / 2
                half_h = label_h
                lon, lat = pf['coords'][0]
                box = (lon - half_w, lat - half_h, lon + half_w, lat + half_h)

                # DEBUG: Log specific cities
                city_name = pf['text'].decode('utf-8', errors='ignore') if isinstance(pf['text'], bytes) else pf['text']
                debug_cities = ['toulouse', 'balma', 'colomiers']
                if zoom == 10 and any(c in city_name.lower() for c in debug_cities):
                    print(f"\n[DEBUG z{zoom}] {city_name}: box={box}, text_len={text_len}, half_w={half_w:.6f}")

                # Check for visual overlap with already placed labels
                overlap = False
                for pb in placed_boxes:
                    if (box[0] < pb[2] and box[2] > pb[0] and
                        box[1] < pb[3] and box[3] > pb[1]):
                        overlap = True
                        if zoom == 10 and any(c in city_name.lower() for c in debug_cities):
                            print(f"[DEBUG z{zoom}] {city_name}: OVERLAP with existing box {pb}")
                        break

                if not overlap:
                    # No visual collision - place this label
                    placed_boxes.append(box)
                    places_placed += 1

                    # Distribute to tiles
                    tiles = get_feature_tiles(pf['coords'], zoom, False)
                    expanded = set()
                    for (tx, ty) in tiles:
                        for dx in range(-1, 2):
                            for dy in range(-1, 2):
                                expanded.add((tx + dx, ty + dy))
                    for tile in expanded:
                        tile_features[tile].append(pf)
                else:
                    # Label overlaps with higher-population city - drop
                    places_dropped_overlap += 1

            # STEP 2: Road labels - try candidate positions, avoid place names
            roads_placed = 0
            roads_dropped = 0

            for rf in road_labels:
                text_len = len(rf['text'])
                half_w = char_w * text_len / 2
                half_h = label_h

                # Try 3 candidate positions along the road
                candidates_to_try = rf.get('coords_candidates', [rf['coords'][0]])
                placed = False

                for candidate_pos in candidates_to_try:
                    lon, lat = candidate_pos
                    box = (lon - half_w, lat - half_h, lon + half_w, lat + half_h)

                    # Check overlap with ALL placed labels (places + roads)
                    overlap = False
                    for pb in placed_boxes:
                        if (box[0] < pb[2] and box[2] > pb[0] and
                            box[1] < pb[3] and box[3] > pb[1]):
                            overlap = True
                            break

                    if not overlap:
                        # Found position without collision
                        rf['coords'] = [(lon, lat)]
                        placed_boxes.append(box)
                        roads_placed += 1
                        placed = True

                        # Distribute to tiles
                        tiles = get_feature_tiles(rf['coords'], zoom, False)
                        expanded = set()
                        for (tx, ty) in tiles:
                            for dx in range(-1, 2):
                                for dy in range(-1, 2):
                                    expanded.add((tx + dx, ty + dy))
                        for tile in expanded:
                            tile_features[tile].append(rf)
                        break

                if not placed:
                    roads_dropped += 1

            if places_dropped_overlap > 0 or roads_dropped > 0:
                print(f"\r  Zoom {zoom:2d}: Labels: {places_placed} places, {roads_placed} roads, {places_dropped_overlap} places dropped (overlap), {roads_dropped} roads dropped")

        # Phase 2: Calculate tile grid from global bbox and write all tiles
        min_tx = lon_to_tile_x(min_lon, zoom)
        max_tx = lon_to_tile_x(max_lon, zoom)
        min_ty = lat_to_tile_y(max_lat, zoom)  # lat is inverted for Y
        max_ty = lat_to_tile_y(min_lat, zoom)

        num_tiles = (max_tx - min_tx + 1) * (max_ty - min_ty + 1)
        if num_tiles <= 0:
            print(f"\r  Zoom {zoom:2d}: No tiles to generate for this area.")
            continue
        
        tiles_written = 0
        print() # New line after preparation phase

        num_workers = min(4, num_tiles)  # Limit to 4 workers to avoid OOM on complex tiles
        tile_jobs = []
        for y in range(min_ty, max_ty + 1):
            for x in range(min_tx, max_tx + 1):
                features = tile_features.get((x, y), [])
                if not features:
                    continue
                tile_dir = os.path.join(output_dir, str(zoom), str(x))
                tile_path = os.path.join(tile_dir, f"{y}.nav")
                features.sort(key=lambda f: f['zoom_priority'] & 0x0F)
                # Debug: log first 5 priorities
                if zoom == 15 and x == min_tx and y == min_ty:
                    print(f"\nDebug tile {x},{y} z{zoom}: First 5 priorities:")
                    for i, f in enumerate(features[:5]):
                        prio = f['zoom_priority'] & 0x0F
                        print(f"  {i}: priority={prio}, type={f['geom_type']}")
                tile_jobs.append((features, tile_path, zoom, x, y, tolerance))

        completed = 0
        with ProcessPoolExecutor(max_workers=num_workers) as executor:
            futures = {executor.submit(write_nav_tile, *job): job for job in tile_jobs}
            for future in as_completed(futures):
                completed += 1
                progress = completed / num_tiles
                bar_width = 25
                filled = int(bar_width * progress)
                bar = '█' * filled + '░' * (bar_width - filled)
                print(f"\r  Zoom {zoom:2d}: Tiles [{bar}] {completed}/{num_tiles}", end='', flush=True)

                result = future.result()
                if result:
                    tiles_written += 1
                    tile_path = futures[future][1]
                    try:
                        total_size += os.path.getsize(tile_path)
                    except OSError:
                        pass

        # Clear memory before next zoom level
        tile_features.clear()
        tile_jobs.clear()

        zoom_elapsed = time.time() - zoom_start
        print(f"\r  Zoom {zoom:2d}: {tiles_written} tiles written. ({zoom_elapsed:.1f}s)" + " " * 20)
        total_tiles += tiles_written

    total_time = time.time() - start_time
    hours, remainder = divmod(int(total_time), 3600)
    minutes, seconds = divmod(remainder, 60)
    if hours > 0:
        time_str = f"{hours}h {minutes:02d}m {seconds:02d}s"
    elif minutes > 0:
        time_str = f"{minutes}m {seconds:02d}s"
    else:
        time_str = f"{total_time:.2f}s"

    logger.info("=" * 50)
    logger.info("Conversion Summary")
    logger.info("=" * 50)
    logger.info(f"Input: {input_pbf}")
    logger.info(f"Output directory: {output_dir}")
    logger.info(f"Format: NAV binary tiles (.nav)")
    logger.info(f"Total tiles: {total_tiles}")
    logger.info(f"Total size: {total_size / (1024 * 1024):.2f} MB")
    logger.info(f"Total time: {time_str}")
    logger.info("=" * 50)

    return total_tiles


def main():
    parser = argparse.ArgumentParser(
        description='Convert OpenStreetMap PBF to NAV binary tile format',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
NAV Format - IceNav Navigation Tiles:
  - int16 relative coordinates (0-4096) for ~50% size reduction
  - Pre-calculated projection for ultra-fast rendering on ESP32
  - BBox-based culling for improved performance
  - Simple binary format optimized for streaming
        """
    )

    parser.add_argument('input_pbf', help='Input PBF file path')
    parser.add_argument('output_dir', help='Output directory for NAV tiles')
    parser.add_argument('config_file', help='Features configuration JSON file')
    parser.add_argument('--zoom', default='6-17',
                        help='Zoom level range (e.g., "6-17" or "12")')
    parser.add_argument('-v', '--verbose', action='store_true',
                        help='Verbose logging (show per-tile filtering stats)')

    args = parser.parse_args()

    if args.verbose:
        logger.setLevel(logging.DEBUG)

    if not os.path.exists(args.input_pbf):
        logger.error(f"Input file not found: {args.input_pbf}")
        sys.exit(1)

    if not os.path.exists(args.config_file):
        logger.error(f"Config file not found: {args.config_file}")
        sys.exit(1)

    if '-' in args.zoom:
        min_zoom, max_zoom = map(int, args.zoom.split('-'))
    else:
        min_zoom = max_zoom = int(args.zoom)

    convert_pbf_to_nav(args.input_pbf, args.output_dir, args.config_file, (min_zoom, max_zoom))


if __name__ == '__main__':
    main()
