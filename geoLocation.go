package main

import (
	"fmt"
	"log"
	"math"
)

// Represents a point on the surface of a sphere. (The Earth is almost
// spherical.) To create an instance, call one of the static methods fromDegrees() or
// fromRadians(). This is a golang port of Java code that was originally published at
// http://JanMatuschek.de/LatitudeLongitudeBoundingCoordinates#Java
//
// Many thanks to the original author: Jan Philip Matuschek
//
// @author Degendra Sivakoti
// @version June 24 2016

const (
	MIN_LAT = -1.5707963267948966 // -PI/2
	MAX_LAT = 1.5707963267948966  //  PI/2
	MIN_LON = -3.141592653589793  // -PI
	MAX_LON = 3.141592653589793   //  PI
)

func degToRad(deg float64) float64 {
	return deg * math.Pi / 180
}

func radToDeg(rad float64) float64 {
	return rad * 180 / math.Pi
}

type GeoLocation struct {
	// latitude in radians
	RadLat float64
	// longitude in radians
	RadLon float64
	// latitude in degrees
	DegLat float64
	// longitude in degrees
	DegLon float64
}

func fromDegrees(latitude, longitude float64) GeoLocation {
	gl := GeoLocation{
		RadLat: degToRad(latitude),
		RadLon: degToRad(longitude),
		DegLat: latitude,
		DegLon: longitude,
	}

	gl.checkBounds()
	return gl
}

func fromRadians(latitude, longitude float64) GeoLocation {
	gl := GeoLocation{
		RadLat: latitude,
		RadLon: longitude,
		DegLat: radToDeg(latitude),
		DegLon: radToDeg(longitude),
	}
	gl.checkBounds()
	return gl
}

func (gl *GeoLocation) checkBounds() {
	if gl.RadLat < MIN_LAT || gl.RadLat > MAX_LAT ||
		gl.RadLon < MIN_LON || gl.RadLon > MAX_LON {
		log.Fatalln("IllegalArgumentException")
	}
}

func (gl *GeoLocation) getLatitudeInDegrees() float64 {
	return gl.DegLat
}

func (gl *GeoLocation) getLongitudeInDegrees() float64 {
	return gl.DegLon
}

func (gl *GeoLocation) getLatitudeInRadians() float64 {
	return gl.RadLat
}

func (gl *GeoLocation) getLongitudeInRadians() float64 {
	return gl.RadLon
}

func (gl *GeoLocation) String() string {
	return fmt.Sprintf("( %f \u00B0, %f \u00B0) = (%f rad, %f rad)", gl.DegLat, gl.DegLon, gl.RadLat, gl.RadLon)
}

// distanceTo computes the great circle distance between this GeoLocation instance
// and the location argument.
// It takes radius, the radius of the sphere, e.g. the average radius for a
// spherical approximation of the figure of the Earth is approximately
// 6371.01 kilometers.
// Returns the distance, measured in the same unit as the radius
// argument.
func (gl *GeoLocation) distanceTo(location GeoLocation, radius float64) float64 {
	return math.Acos(math.Sin(gl.RadLat)*math.Sin(location.RadLat)+
		math.Cos(gl.RadLat)*math.Cos(location.RadLat)*
			math.Cos(gl.RadLon-location.RadLon)) * radius
}

// boundingCoordinates computes the bounding coordinates of all points on the surface
// of a sphere that have a great circle distance to the point represented
// by this GeoLocation instance that is less or equal to the distance
// argument.
// For more information about the formulae used in this method visit
// http://JanMatuschek.de/LatitudeLongitudeBoundingCoordinates
// It takes distance, the distance from the point represented by this
// GeoLocation instance. Must me measured in the same unit as the radius
// argument, and radius, the radius of the sphere, e.g. the average radius for a
// spherical approximation of the figure of the Earth is approximately
// 6371.01 kilometers.
// Returns an array of two GeoLocation objects such that:
// 1. The latitude of any point within the specified distance is greater
// or equal to the latitude of the first array element and smaller or
// equal to the latitude of the second array element.
// 2. If the longitude of the first array element is smaller or equal to
// the longitude of the second element, then
// the longitude of any point within the specified distance is greater
// or equal to the longitude of the first array element and smaller or
// equal to the longitude of the second array element.
// 3. If the longitude of the first array element is greater than the
// longitude of the second element (this is the case if the 180th
// meridian is within the distance), then
// the longitude of any point within the specified distance is greater
// or equal to the longitude of the first array element
// or smaller or equal to the longitude of the second
// array element.
func (gl *GeoLocation) boundingCoordinates(distance, radius float64) []GeoLocation {

	if radius < 0 || distance < 0 {
		log.Fatalln("IllegalArgumentException")
	}

	// angular distance in radians on a great circle
	radDist := distance / radius

	minLat := gl.RadLat - radDist
	maxLat := gl.RadLat + radDist

	var minLon, maxLon float64
	if minLat > MIN_LAT && maxLat < MAX_LAT {
		deltaLon := math.Asin(math.Sin(radDist) / math.Cos(gl.RadLat))
		minLon = gl.RadLon - deltaLon
		if minLon < MIN_LON {
			minLon += 2 * math.Pi
		}
		maxLon = gl.RadLon + deltaLon
		if maxLon > MAX_LON {
			maxLon -= 2 * math.Pi
		}
	} else {
		// a pole is within the distance
		minLat = math.Max(minLat, MIN_LAT)
		maxLat = math.Min(maxLat, MAX_LAT)
		minLon = MIN_LON
		maxLon = MAX_LON
	}
	return []GeoLocation{
		fromRadians(minLat, minLon),
		fromRadians(maxLat, maxLon),
	}
}

func main() {
	LAT_DEG := 40.689604
	LON_DEG := -74.04455
	LAT_DEG2 := 38.890298
	LON_DEG2 := -77.035238
	//DISTANCE_MI := 201.63714020616294
	//DISTANCE_KM = 324.503521805324

	gl := fromDegrees(LAT_DEG, LON_DEG)
	gl2 := fromDegrees(LAT_DEG2, LON_DEG2)

	log.Println(gl.distanceTo(gl2, 6371.01))
}
