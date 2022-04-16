//
//  ViewController.swift
//  StepsProcessor
//
//  Created by Nikita on 04/16/2022.
//  Copyright (c) 2022 Nikita. All rights reserved.
//

import UIKit
import StepsProcessor

class ViewController: UIViewController {

    override func viewDidLoad() {
        super.viewDidLoad()
        
        DispatchQueue.global(qos: .utility).async {
            
            print("Reading test file...")
            
            guard let fileUrl = Bundle.main.url(forResource: "RawInput", withExtension: "json") else {
                print("Can`t find test file!")
                return
            }
            
            guard let data = try? Data(contentsOf: fileUrl) else {
                print("Can`t open test file!")
                return
            }
            
            guard let input = try? JSONDecoder().decode([RawInput].self, from: data) else {
                print("Can`t decode test file!")
                return
            }
            
            if #available(iOS 13.0, *) {
                let s = CACurrentMediaTime()
                let data = try? StepsProcessor.process(input: input)
                let delta = CACurrentMediaTime() - s
                
                guard let data = data else {
                    print("Can`t process test file!")
                    return
                }
                
                print("Calculation time:", delta)
                print("Average step length:", data.averageLength)
                print("Average step height:", data.averageHeight)
                print("Steps count:", data.stepsCount)
            }
        }
    }

    override func didReceiveMemoryWarning() {
        super.didReceiveMemoryWarning()
        // Dispose of any resources that can be recreated.
    }

}

