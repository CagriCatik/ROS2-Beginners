import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

const FeatureList = [
  {
    title: 'What is ROS 2?',
    description: (
      <>
        ROS 2 (Robot Operating System 2) is an open-source framework that provides tools and libraries to build and control robots. 
        It introduces modern middleware and communication protocols, making it an ideal choice for robotics development.
      </>
    ),
  },
  {
    title: 'Why Choose ROS 2?',
    description: (
      <>
        ROS 2 offers cross-platform support, real-time capabilities, and improved security compared to its predecessor. 
        It’s designed for both beginners and advanced developers looking to create scalable and robust robotic systems.
      </>
    ),
  },
  {
    title: 'Getting Started',
    description: (
      <>
        Start with the basics by installing ROS 2 on your system, exploring core concepts like nodes, topics, services, and actions, 
        and building your first publisher-subscriber example. The learning curve is friendly for beginners.
      </>
    ),
  },
];

function Feature({title, description}) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
