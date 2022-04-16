//
//  IMUInput.swift
//  StepsProcessor
//
//  Created by Ирина Токарева on 16.04.2022.
//

import Foundation

public protocol IMUInput {
    var ax: Float { get }
    var ay: Float { get }
    var az: Float { get }
    var gx: Float { get }
    var gy: Float { get }
    var gz: Float { get }
}
