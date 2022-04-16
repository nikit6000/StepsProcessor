//
//  StepsProcessor.swift
//  Pods-StepsProcessor_Example
//
//  Created by Ирина Токарева on 16.04.2022.
//

import Foundation
import Accelerate
import FiltFilt
import simd

public enum StepsProcessor {
    
    private struct Quaternion {
        let q0: Float
        let q1: Float
        let q2: Float
        let q3: Float
    }
    
    private static let frequency: Double = 100.0
    
    // MARK: - Public methods
    
    @available(iOS 13.0, *)
    public static func process(input: [IMUInput]) throws -> StepsInfo {
        
        let time = (0..<input.count).map {
            Double($0) / frequency
        }
        
        let accMag = input.map {
            Double(sqrt($0.ax * $0.ax +  $0.ay *  $0.ay +  $0.az *  $0.az))
        }
        
        let highPassFilterParameters = FiltFiltSwift.Parameters(
            a: [1, -0.999937170120766],
            b: [0.999968585060383, -0.999968585060383]
        )

        let lowPassFilterParameters = FiltFiltSwift.Parameters(
            a: [1, -0.726542528005361],
            b: [0.136728735997320, 0.136728735997320]
        )
        
//        256hz
//        let highPassFilterParameters = FiltFiltSwift.Parameters(
//            a: [1, -0.999975456608585],
//            b: [0.999987728304293, -0.999987728304293]
//        )
//
//        let lowPassFilterParameters = FiltFiltSwift.Parameters(
//            a: [1, -0.884239215225350],
//            b: [0.0578803923873251, 0.0578803923873251]
//        )
        
        let highPassFilter = try FiltFiltSwift(parameters: highPassFilterParameters)
        let lowPassFilter = try FiltFiltSwift(parameters: lowPassFilterParameters)
        
        let accMagFiltAbsHP = highPassFilter.process(input: accMag).map {
            abs($0)
        }
        
        let accMagFilt = lowPassFilter.process(input: accMagFiltAbsHP)
        let stationaty = accMagFilt.map { $0 < 0.05 }
        
        let initPeriod: Double = 2
        var indexSel = 0..<time.count
        
        for i in indexSel {
            if (time[i] - (time[0] + initPeriod)) > 0 {
                indexSel = 0..<i
                break
            }
        }
        
        // Reset AHRS to initial state
        q0 = 1
        q1 = 0
        q2 = 0
        q3 = 0
        twoKp = 1
        twoKi = 0
        
        for _ in 0..<2000 {
            let meanAx = vDSP.mean(input[indexSel].map { $0.ax } )
            let meanAy = vDSP.mean(input[indexSel].map { $0.ay } )
            let meanAz = vDSP.mean(input[indexSel].map { $0.az } )
            
            MahonyAHRSupdateIMU(0, 0, 0, meanAx, meanAy, meanAz)
        }
        
        var quat = [Quaternion]()
        
        for i in 0..<input.count {
            if stationaty[i] {
                twoKp = 0.5
            } else {
                twoKp = 0
            }
            let element = input[i]
            let gx = deg2rad(element.gx)
            let gy = deg2rad(element.gy)
            let gz = deg2rad(element.gz)
            
            MahonyAHRSupdateIMU(gx, gy, gz, element.ax, element.ay, element.az)
            quat.append(.init(q0: q0, q1: q1, q2: q2, q3: q3))
        }
        
        let acc = input.enumerated().map { t -> SIMD3<Float> in
            let quaternion = quat[t.offset]
            let sq = simd_quaternion(
                quaternion.q0,
                quaternion.q1,
                quaternion.q2,
                quaternion.q3
            )
            let gravity: Float = 9.81
            let accVec = t.element
            let vector = SIMD3<Float>(
                x: accVec.ax,
                y: accVec.ay,
                z: accVec.az
            )
            let rotatedVector = sq.act(vector * gravity)
            
            return SIMD3<Float>(x: rotatedVector.x, y: rotatedVector.y, z: rotatedVector.z - gravity)
        }
        
        let sampleRatePeriod = 1.0 / Float(frequency)
        var vel = [SIMD3<Float>](
            repeating: .init(x: 0, y: 0, z: 0),
            count: acc.count
        )
        
        for i in 1..<acc.count {
            var v = vel[i - 1] + acc[i] * sampleRatePeriod
            if stationaty[i] {
                v = .zero
            }
            vel[i] = v
        }
        
        var velDrift = [SIMD3<Float>](
            repeating: .init(x: 0, y: 0, z: 0),
            count: acc.count
        )
        
        let stationaryDiff = stationaty.diff().enumerated()
        var stationaryStart = stationaryDiff.compactMap { t -> Int? in
            guard t.element == -1 else {
                return nil
            }
            return t.offset
        }
        var stationaryEnd = stationaryDiff.compactMap { t -> Int? in
            guard t.element == 1 else {
                return nil
            }
            return t.offset
        }
        
        if stationaryStart.count != stationaryEnd.count {
            if stationaryEnd[0] < stationaryStart[0] {
                stationaryEnd.removeFirst()
            }
            
            if stationaryStart[stationaryStart.count - 1] > stationaryEnd[stationaryEnd.count - 1] {
                stationaryStart.removeLast()
            }
        }
        
        for i in 0..<stationaryEnd.count {
            let diffRate = vel[stationaryEnd[i] - 1] / Float(stationaryEnd[i] - stationaryStart[i])
            let en = 0...(stationaryEnd[i] - stationaryStart[i])
            let drift = en.map {
                diffRate * Float($0)
            }
            for k in stationaryStart[i]..<stationaryEnd[i] {
                velDrift[k] = drift[k - stationaryStart[i]]
            }
        }
        
        // Remove integral drift
        
        for i in 0..<vel.count {
            vel[i] -= velDrift[i]
        }
        
        var pos = [SIMD3<Float>](
            repeating: .init(x: 0, y: 0, z: 0),
            count: vel.count
        )
        
        for i in 1..<pos.count {
            pos[i] = pos[i - 1] + vel[i] * sampleRatePeriod
        }
        
        let stepsCount = stationaryEnd.count
        var averageLength: Float = 0
        var averageHeight: Float = 0
        
        for i in 0..<stepsCount {
            let startIndex = stationaryStart[i]
            let endIndex = stationaryEnd[i]
            
            let startPos = pos[startIndex]
            let endPos = pos[endIndex]
            
            let maxHeightPont = pos[startIndex...endIndex].max(
                by: {
                    $0.z < $1.z
                }
            )!
            
            let stepLineVector = endPos - startPos
            let lineVec = startPos - maxHeightPont
            
            averageHeight += length(cross(lineVec, stepLineVector)) / length(stepLineVector)
            averageLength += distance(startPos, endPos)
        }
        
        averageLength /= Float(stepsCount)
        averageHeight /= Float(stepsCount)
        
        return .init(
            averageLength: averageLength,
            averageHeight: averageHeight,
            stepsCount: Float(stepsCount)
        )
    }

    // MARK: - Private methods
    
    private static func deg2rad(_ number: Float) -> Float {
        return number * .pi / 180.0
    }
}

private extension Array where Element == Bool {
    
    func diff() -> [Int] {
        var output = [Int].init(repeating: 0, count: count)
        
        for i in 0..<(count - 1) {
            let next: Int = self[i + 1] ? 1 : 0
            let curr: Int = self[i] ? 1 : 0
            
            output[i + 1] = next - curr
        }
        
        return output
    }
}
