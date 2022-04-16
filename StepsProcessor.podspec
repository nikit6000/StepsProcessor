#
# Be sure to run `pod lib lint StepsProcessor.podspec' to ensure this is a
# valid spec before submitting.
#
# Any lines starting with a # are optional, but their use is encouraged
# To learn more about a Podspec see https://guides.cocoapods.org/syntax/podspec.html
#

Pod::Spec.new do |s|
  s.name             = 'StepsProcessor'
  s.version          = '0.1.0'
  s.summary          = 'A short description of StepsProcessor.'

  s.description      = <<-DESC
TODO: Add long description of the pod here.
                       DESC

  s.homepage         = 'https://github.com/Nikita/StepsProcessor'
  # s.screenshots     = 'www.example.com/screenshots_1', 'www.example.com/screenshots_2'
  s.license          = { :type => 'MIT', :file => 'LICENSE' }
  s.author           = { 'Nikita' => 'tarhov.nik@gmail.com' }
  s.source           = { :git => 'git@github.com:nikit6000/StepsProcessor.git', :tag => s.version.to_s }
  
  s.ios.deployment_target = '9.0'
  s.source_files = ['StepsProcessor/Classes/**/*.swift', 'StepsProcessor/Classes/IMU/*.c', 'StepsProcessor/Classes/IMU/*.h']
  s.frameworks = 'Foundation'
  s.dependency 'FiltFilt', '~> 0.1.0'
end
