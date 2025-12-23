import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <Heading as="h1" className="hero__title">
          {siteConfig.title}
        </Heading>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/Physical-AI-Humanoid-Robotics/">
            Start Learning 🚀
          </Link>
        </div>
      </div>
    </header>
  );
}

type ModuleItem = {
  title: string;
  description: string;
  link: string;
};

const ModuleList: ModuleItem[] = [
  {
    title: 'Module 1: The Robotic Nervous System (ROS 2)',
    description: 'Learn how ROS 2 serves as the "nervous system" for humanoid robots. Master node communication, Python integration with rclpy, and robot description with URDF.',
    link: '/docs/Physical-AI-Humanoid-Robotics/ros2-nervous-system/',
  },
  {
    title: 'Module 2: Sensors & Perception Systems',
    description: 'Discover how humanoid robots sense and understand their environment through cameras, depth sensors, IMUs, and advanced sensor fusion techniques.',
    link: '/docs/Physical-AI-Humanoid-Robotics/sensors-perception/',
  },
  {
    title: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
    description: 'Explore the NVIDIA Isaac platform for photorealistic simulation, hardware-accelerated perception, and specialized navigation for bipedal robots.',
    link: '/docs/Physical-AI-Humanoid-Robotics/isaac-ai-brain/',
  },
  {
    title: 'Module 4: Vision-Language-Action Models',
    description: 'Build end-to-end autonomous behavior using foundation models, vision-language integration, and cognitive planning for humanoid robots.',
    link: '/docs/Physical-AI-Humanoid-Robotics/vision-language-action/',
  },
];

function Module({title, description, link}: ModuleItem) {
  return (
    <div className={clsx('col col--6', styles.moduleCard)}>
      <div className="card margin--md">
        <div className="card__header">
          <h3>{title}</h3>
        </div>
        <div className="card__body">
          <p>{description}</p>
        </div>
        <div className="card__footer">
          <Link className="button button--primary button--block" to={link}>
            Open Module →
          </Link>
        </div>
      </div>
    </div>
  );
}

function HomepageModules() {
  return (
    <section className={styles.modules}>
      <div className="container">
        <div className="row margin-vert--lg">
          <div className="col">
            <Heading as="h2" className="text--center margin-bottom--lg">
              What This Textbook Covers
            </Heading>
            <p className="text--center margin-bottom--xl" style={{fontSize: '1.1rem'}}>
              A comprehensive journey from foundational robotic systems to cutting-edge AI integration.
              Each module builds upon the last to create a complete understanding of humanoid robotics.
            </p>
          </div>
        </div>
        <div className="row">
          {ModuleList.map((props, idx) => (
            <Module key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}

function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row margin-vert--lg">
          <div className="col col--4">
            <div className="text--center padding--md">
              <h3>💬 AI-Native Learning</h3>
              <p>Integrated AI colearning prompts throughout each lesson to deepen understanding with Claude or ChatGPT.</p>
            </div>
          </div>
          <div className="col col--4">
            <div className="text--center padding--md">
              <h3>🎓 Expert Insights</h3>
              <p>Learn best practices, common pitfalls, and industry-standard approaches from experienced practitioners.</p>
            </div>
          </div>
          <div className="col col--4">
            <div className="text--center padding--md">
              <h3>🤝 Hands-On Practice</h3>
              <p>Apply concepts through design challenges, capstone projects, and real-world integration exercises.</p>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Home`}
      description="Learn the complete technology stack for building intelligent humanoid robots with Physical AI">
      <HomepageHeader />
      <main>
        <HomepageModules />
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
