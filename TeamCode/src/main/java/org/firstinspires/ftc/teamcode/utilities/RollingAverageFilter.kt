package org.firstinspires.ftc.teamcode.utilities

class RollingAverageFilter(private val window: Int) {
    private val values = mutableListOf<Double>()
    init {
        for(i in 0 until window){
            values.add(0.0)
        }
    }

    fun add(num: Double){
        if(values.size >= window){
            values.removeAt(0)
        }
        values.add(num)
    }

    fun get(): Double{
        return values.average()
    }


}