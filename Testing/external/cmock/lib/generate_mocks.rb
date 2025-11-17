require 'fileutils'

# destination path for the mock files is specified in config.yml, which is located in the same folder as generate_mocks.rb

headers_path = '../../../../RocketLib/Utils/Inc'
config_file = 'config.yml'
cmock_path   = 'cmock.rb'

Dir.glob("#{headers_path}/*.h").each do |header|
  puts "Generating mock for #{header}"
  system("ruby #{cmock_path} -o#{config_file} #{header}")
end