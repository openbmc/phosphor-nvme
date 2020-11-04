executable(
    'nvme_power_control',
    [
        'nvme_power_control.cpp',
    ],
    dependencies: [
        dependency('boost'),
        dependency('sdbusplus'),
        dependency('libgpiod'),
        dependency('libgpiodcxx'),
        dependency('threads'),
    ],
    install: true,
    install_dir: get_option('bindir')
)

configfile = 'nvme_power_gpio.json'
confpath = '"' + join_paths(
    '/etc/nvme',
    configfile,
) + '"'

install_data(sources : configfile, install_dir : '/etc/nvme')

conf_data = configuration_data()
conf_data.set('NVME_POWER_GPIO_CONFIG_FILE', confpath)

configure_file(output : 'config.hpp',
               configuration : conf_data)

systemd = dependency('systemd')
conf_data = configuration_data()
conf_data.set('bindir', get_option('prefix') / get_option('bindir'))
configure_file(
  input: 'xyz.openbmc_project.Control.Nvme.Power.service.in',
  output: 'xyz.openbmc_project.Control.Nvme.Power.service',
  configuration: conf_data,
  install: true,
  install_dir: systemd.get_pkgconfig_variable('systemdsystemunitdir'))
