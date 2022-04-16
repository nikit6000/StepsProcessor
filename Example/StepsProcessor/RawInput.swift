import Foundation
import StepsProcessor

struct RawInput: IMUInput, Decodable {
	let ax: Float
	let ay: Float
	let az: Float
	let gx: Float
	let gy: Float
	let gz: Float
}
